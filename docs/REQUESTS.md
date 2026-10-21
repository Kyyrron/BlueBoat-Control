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

**Status 2026-10-19: applied** by the user's order. The rows went into root §4.1 (plus a §4.2 note), the §3 paragraph into root §3, and D8 into root §6. The pathlog went into the `file-interfaces` skill's mission-report row. The `/blueboat/update_path` row names today's clients honestly: only this module's tests. The replanner and an MCS Replanner mode do not exist yet.

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
