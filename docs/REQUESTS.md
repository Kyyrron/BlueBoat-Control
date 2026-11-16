# Requests to other modules — live path update

The live path update ([PATH_UPDATE_INTERFACE.md](PATH_UPDATE_INTERFACE.md)) was built inside
BlueBoat-Control only. Nothing outside this module was edited.

The items below are what other modules need for the feature to be usable end to end. Each one
is a request for the owner of that module to schedule.

## R-1 BlueBoat-MCS — "Replanner mode" (updatable-path mission)

MCS is frozen at v1.0, so this needs an explicit order.

**Need.** The operator must be able to choose, per run, between:
- **fixed mission** — today's behaviour, unchanged and still the default;
- **updatable path** — a path driven by BlueBoat-Replanner.

**Usable today, before any MCS change.** The Launch dialog's **Extra args** field is passed
verbatim to the launch file (`launch_manager.to_cli`):
- Typing `path_source:=replanner` there starts a replanner **simulation** (`Sim_launch.py`).
- Or it starts a real-boat **session** (`BlueBoat_launch.py`) whose runs are all
  replanner-owned. MCS's `configure` omits `path_source`, which keeps the launch default.

MCS still writes `run.yaml`; Control ignores it for such runs. What MCS lacks is the per-run
choice and the progress display.

**What Control already accepts:**

1. **Run sessions.** Add `"path_source": "replanner"` to the `configure` command on
   `/blueboat/run_command`.
   - Absent or `"mission"` means a fixed mission, exactly as today.
   - Control refuses `"replanner"` together with `use_pinger: true`.
   - The current value is echoed in `/blueboat/run_state` as `"path_source"`.
2. **Simulation launches.** Pass `path_source:=replanner` to `Sim_launch.py`.
3. **Real launches.** Optionally pass `path_source:=replanner` to `BlueBoat_launch.py`. It is
   the session's default and `configure` overrides it.

**Suggested behaviour in replanner mode:**
- **Session file.** Do not write or deploy `trajectories/.session/run.yaml` for such a run.
  Control does not read it and does not snapshot it.
- **Display.** Subscribe `/blueboat/path_progress` (JSON) and show:
  - `state`: `awaiting_path` / `following` / `end_reached` / `override` / …;
  - `following_id`;
  - `tau` / `t_end`;
  - the last refusal (`last_answer.reason`, `last_answer.message`).
- **Map.** Optional: draw the hold pose while `awaiting_path`.
- **Safety.** Keep every safety control unchanged. Control already refuses updates while
  E-STOP, manual target or arrows are engaged.
- **Smoke test.** Add one case: a replanner session run that starts, holds, and stops with no
  path.

## R-2 BlueBoat-Replanner (new module)

Code against [PATH_UPDATE_INTERFACE.md](PATH_UPDATE_INTERFACE.md), in particular:
- §10.4, the loop;
- §5, how to react to each refusal category.

`test/path_update_demo.py` in this module is a worked client written against that document
only.

Duties the Replanner owns:
- path feasibility: speed and turn radius;
- unique ids per run;
- adding its own latency margin to `min_change_point`;
- its own copy of the accepted path, evaluated with the §3 rule.

The three design rules of `project_synthesis.md` §8.1 (resolution, budget, abandonment) are
the Replanner's, not Control's.

## R-3 BlueBoat-SSS-Sim — `full_mission_launch.py`

That launch starts its own copies of Control's nodes, so built-world simulations cannot run
replanner-owned runs yet.

**Request.** Add a `path_source` argument, default `mission`, and pass it as a parameter to
its `master_control` node.

Nothing else is needed. The simulator's acquisition and sea-state paths are unaffected.

## R-4 Superproject root `.claude/CLAUDE.md` — interface tables (for the user to apply)

**Status 2026-10-19: applied** by the user's order. The rows went into root §4.1 (plus a §4.2 note), the §3 paragraph into root §3, and D8 into root §6. The pathlog went into the `file-interfaces` skill's mission-report row. The `/blueboat/update_path` row names today's clients honestly: only this module's tests. The replanner and an MCS Replanner mode do not exist yet. (2026-11-13: BlueBoat-Replanner exists and is a client; it has not yet run against `master_control`. The MCS Replanner mode is still R-1.)

**§4.1, new rows:**

| Name | Type | Producer → Consumers | Notes |
|---|---|---|---|
| `/blueboat/update_path` | service `blueboat_interfaces/srv/UpdatePath` | Replanner, MCS, tests → `master_control` | **Live path update (2026-10-15)**, additive, in every launch sim and real. Replaces the rest of a **replanner-owned** run's path from `change_point` on, or loads its first path. Fixed-mission runs answer `mission_owned`. Contract: `BlueBoat-Control/docs/PATH_UPDATE_INTERFACE.md` |
| `/blueboat/path_progress` | `std_msgs/String` (JSON, `blueboat_path_progress/1`) | `master_control` → Replanner, MCS | 10 Hz + on change, default QoS. τ (the governor's path parameter) in every run; state, following/active ids, `min_change_point`, end of path |
| `/blueboat/path_update_event` | `std_msgs/String` (JSON, `blueboat_path_update_event/1`) | `master_control` → `robot_interface`, `simulation_interface`, bag | Reliable, depth 50. One message per answer / activation / end / clear. Logged as the run's `<date>-<note>-pathlog.csv` |

**§4.1 rows to amend:**
- `/blueboat/run_command`: `configure` gains the optional `path_source` field
  (`mission` | `replanner`).
- `/blueboat/run_state`: gains the additive key `path_source`.

**§4.5:** add the run's `-pathlog.csv` to the file-interfaces skill. It sits beside the poslog
and is filed into `Robot_data/<run>/`; it exists only if the run received a path update
request.

**§6, new entry D8 — mission swap:**
> `project_synthesis.md` §5.1 has the mission manager perform a "MAVLink mission swap" to
> ArduPilot. In the implementation nothing goes to ArduPilot: `master_control` follows the path
> on the laptop and streams RC override (Control N6). The live path update
> (`/blueboat/update_path`, 2026-10-15) is the actual mechanism for that role. *More
> reliable:* the module; §5.1 reads as a pre-implementation sketch.

**§3, Control's ordered-changes paragraph:**
> **On 2026-10-15 the user ordered the live path update in Control** (Replanner entry point):
> - New service `/blueboat/update_path` (`UpdatePath.srv`) and topics `/blueboat/path_progress`
>   and `/blueboat/path_update_event`.
> - New parameter / launch argument `path_source` (default `mission`) and `path_update_*`
>   thresholds.
> - New `configure` field and `run_state` key `path_source`.
> - A run's `-pathlog.csv`.
>
> Additive only; fixed-mission behaviour is proven bit-identical by `test/mc_harness.py
> --compare HEAD`.

## R-5 Optional — visualisation of the updated path

Currently nothing draws the geometry of an updated path:
- `/set_path` is drawn by `path_publisher` (simulation only) from `path_generation`;
- bag replay draws the run-start `-path.yaml`.

The followed reference is in the poslog target columns and the events.

If MCS or the GCS want the geometry, Control can add a latched `nav_msgs/Path` topic of the
plan, e.g. `/blueboat/path_update/plan`. Ask, and it will be a small additive change.

Since 2026-11-12, in a replanner-owned run, BlueBoat-Replanner publishes `/replanner/plan`
(latched `nav_msgs/Path`, frame `world`): its mirror of the path Control holds, republished on
every accepted change. Display only; Control's own topic would still be the authority.

---

# Requests to other modules — identified plant and field-equivalent simulation (2026-11-13)

Session 04 of the simulator completion round changed BlueBoat-Control only. Evidence:
`~/ros2_ws/sim_runs/sim_acceptance/04_control/` (`handoff_04.md`); numbers:
[PARAMETER_REGISTER.md](PARAMETER_REGISTER.md).

## R-6 BlueBoat-SSS-Sim — `full_mission_launch.py`: forward the simulation configuration

**File.** `blueboat_sss_sim/launch/full_mission_launch.py` (it builds Control's stack itself).

**Today.** It includes `upload_rov_launch.py` with `{sliders, thr}` and starts
`simulation_interface` and `master_control` with fixed parameter lists. Every piece therefore
takes its new default: identified plant, raw odometry bridge, real-boat column, 20 Hz thrust,
`field_13hz` odometry. That is the field-equivalent configuration, coherent, with nothing to
change. What cannot be had from that launch is the **legacy** configuration.

**Change.** Declare `sim_config` (`field_equivalent` | `legacy`) and pass, for `legacy`:
`plant:=bluerov2_legacy` and `odom_raw:=false` to `upload_rov_launch.py`;
`thrust_rate_hz: 10.0`, `odom_profile: passthrough` to `simulation_interface`;
`gain_profile: sim` to `master_control`. Optionally `tuning_file`, `odom_profile`,
`odom_seed` as `Sim_launch.py` does (`SIM_CONFIGS` there is the reference).

**Reason.** Reproducing a pre-2026-11-13 world run; re-flying a field run with that day's gains.

## R-7 BlueBoat-SSS-Sim — pose fallbacks now see field-rate odometry

**Files.** `ros/sea_state_node.py`, `ros/sss_sim_node.py` (parameter `odom_topic`, default
`/blueboat/odom`), `ros/mavros_shim_node.py`.

**What changed.** In the field-equivalent configuration `/blueboat/odom` is no longer Gazebo's
clean 20 Hz: it arrives as on the boat (4 Hz stretches, 25 Hz bursts, a few ms to 160 ms late).
Gazebo's own odometry is on the new sim-only topic **`/blueboat/odom_gz`** (same type, 20 Hz).

**Change.** Where a node needs the boat's true pose rather than what the boat's software sees
(the sea-state wrench and the sonar renderer when `/sim/sea_state/boat_pose` is stale), point
its `odom_topic` at `/blueboat/odom_gz`. The MAVROS shim should stay on `/blueboat/odom`: the
real `/mavros/*` stream is what `/blueboat/odom` is made from.

**Reason.** The simulator's own note: a stale pose fed to the hydrostatic wrench pumps heave.

## R-8 BlueBoat-SSS-Sim — heave / roll / pitch entries and the mirrored hull numbers

**Files.** `config/sea_states.yaml` (`hull:`), `sea/motion.py` (`URDF_*`, `PLUGIN_*`),
`docs/sea_state_model.md`.

1. The identified plant keeps `zDotW`, `kDotP`, `mDotQ`, `zW`, `kP`, `mQ`, `zWabsW`, `kPabsP`,
   `mQabsQ` at the BlueROV2 values **only because** `hull.damping` is sized as their complement
   and `sea/motion.py` mirrors them. They are another vehicle's numbers (acceptance test I4).
   Once the simulator owns the whole vertical model, tell Control which values to carry (zero is
   the natural choice) and they will be changed in `hydrodynamics_identified.xacro` together.
2. **Yaw inertia of the link changed**: `iyy` (the link's yaw axis, the inertial frame being
   rolled by π/2) is 7.50 kg·m² in the identified plant, 3.9235025 in the legacy one. The
   first `<inertial>` block of `blueboat.xacro` is still the legacy literal, so smoke test
   `[10]` passes unchanged; `URDF_LINK_INERTIA` in `sea/motion.py` describes the legacy plant.
3. **Mass.** 16.01 kg stays (the box buoyancy is subtracted with it). The vendor's figures
   put a survey-ready boat at 17.5 kg (two battery packs, two sonar heads) to 19.8 kg (four
   packs) before mounts and cables. It will not be weighed (R-10): 16.01 kg stays, 16–20 kg is
   the bracket.
4. **Demihull spacing.** The vendor CAD gives keels 583 mm apart (±10 mm, measured on the mesh);
   the simulator's catamaran hydrostatics use 0.76 m. To check on the boat.
5. **Plugin sign.** gz-sim 8.15's hydrodynamics plugin applies the added-mass Coriolis terms
   with the opposite sign to Fossen's `C_A` in every plane (`HydrodynamicsUtils.hh`,
   `buildCoriolisMatrix`, applied as `+Cmat·ν`). In the horizontal plane Control fitted its yaw
   damping on the plant as integrated, so the response is the identified one. In the vertical
   plane the same terms act on the simulator's heave / pitch (`zDotW` 14.57 kg × surge speed).

**Answered 2026-11-15 (session 04b; session 03's `03_sea_state/R8_request_for_control.md`;
`~/ros2_ws/sim_runs/sim_acceptance/04b_control_r8/handoff_04b.md`).** Applied, with three
values the plugin cannot integrate carried differently:
- `zDotW` is −14.5, not −16.15 (FORCED: at 16.15 the boat diverged in Gazebo).
- A44 1.79 and A55 0.93 are rigid roll / pitch inertia (`ixx` 4.21369, `izz` 6.5703125) with
  `kDotP` / `mDotQ` 0 (FORCED: the coupled sway-roll mode diverged).
- The vertical damping is 0 under the new xacro argument `vertical_model:=sea_node` (default: the
  sea node owns it) and carried by the plugin under `standalone` (`Sim_launch.py`).

Also changed: the CoG under `sea_node` is the derived physical one, +0.1376 at link z (was −0.129,
below the keel), and the sway drag is re-derived at the CAD draft (`yVabsV` −319.1, `nR` −6.35,
`nRabsU` −5.12). Register §3–§4.

What the simulator has to follow up:
- `sea/plant.py` `IDENTIFIED` mirror and `plant_from_xacro` (CoG property `z_cog_physical`, the
  inertia block now differs from the legacy one in `ixx` / `izz` too);
- smoke `[10]`;
- `derive_r8.py` (CoG, the A33 cap);
- the free-decay proxy and the `harbour_chop` estimate.

## R-9 Session 03 (sea state) — what is left to the disturbance

- The plant carries **no** right/left thrust asymmetry: not identifiable in the pool
  (−0.24 … +0.07 by model form). Every 2026-09-07 FIT run needed +2.2 to +3.8 N more on the
  right, in wind. A refit on them gives an asymmetry of −0.19 … −0.68 or, as well, a constant
  yaw torque of −0.13 … −0.31 N·m (physical units, right-turning). That bias belongs to the
  field-matched disturbance. Knobs on Control's side: the two `thrust_gain_*` tags, or the
  `thrust_gain` parameter of `simulation_interface`.
- The pool surge model over-predicts the 2026-09-07 mean speeds by 3–14 % (PID, MPC) and
  24–41 % (LoS, 3–4 N of total thrust) without wind; a constant world force of 1–3 N towards
  S/SW brings all six to 0.08–0.12 m/s rms.
- κ (0.493, bracket 0.45–0.85) sets how hard a wave or current force moves the boat for a given
  response to thrust. If the roll / yaw disturbance fit wants a different κ, it is a legitimate
  free parameter of that fit: change both `thrust_gain_*` tags and scale `xUabsU`, `nR`,
  `nRabsU`, `iyy + nDotR`, `xDotU + m` by the same factor (the response to commands is then
  unchanged).

## R-10 No further field measurement is planned (owner, 2026-11-14)

The boat will not be weighed and no new run will be recorded. What the logs could not fix therefore stays
a **bracket**, to be varied in the sensitivity study rather than closed:

| Quantity | Carried value | Bracket | Knob |
|---|---|---|---|
| κ, commanded → applied Newton (rests on the mass) | 0.493 | 0.45 – 0.85 | `thrust_gain_*` tags, coefficients scaled with it (R-9) |
| Right/left thrust asymmetry `a` | 0 | −0.25 … +0.10 | `thrust_gain` |
| Sway added mass / drag | 14 kg / 319.1 N·s²/m² (2026-11-15: drag re-derived at the CAD draft) | 8 – 133 (Gazebo ≤ 14, plugin stability) / 213 – 426 | `yDotV`, `yVabsV` (then refit `nR`, `nRabsU`: `04b_control_r8/fit/yaw_fit5_r8_fixIzz.py`) |
| CoG height (`vertical_model:=sea_node`) | link z +0.1376 (0.239 m above the keel) | +0.097 … +0.178 | `z_cog_physical` (`blueboat.xacro`) |
| Heave added mass | 14.5 kg (FORCED cap) | R-8: 8.1 – 24.2 kg; Gazebo < 16.15 | `zDotW_identified` |
| Mass | 16.01 kg | 16 – 20 kg | not to be changed alone (the simulator mirrors it) |
| Constant odometry delay | 0 s | 0 – 0.10 s | `odom_base_delay` |

None of them moves the response to commanded thrust, which is what was identified.
