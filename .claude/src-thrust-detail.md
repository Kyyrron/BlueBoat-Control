# blueboat_control/src — thrust path detail (§5) and bag-replay evidence

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## §5 Thrust path — detail

- `/thruster_input` carries **`[right, left]`**. The convention is consistent across every
  code path: the allocation matrix `B = [[1,1],[0,0],[r,-r]]` with `radius = 0.59/2` puts a
  positive (CCW) yaw moment on column 0; the URDF places `thruster1` at `y = -0.295`
  (starboard) and `thruster2` at `y = +0.295` (port), and the yaw moment of a body-x force at
  `y` is `-y*Fx`, reproducing `+0.295 / -0.295` exactly; `ROV.read_model` sorts thruster
  joints alphabetically, so `forces[0]` drives `thruster1`; `simulation_interface` unpacks
  `r, l = thr_input`; `solve_LoS` builds `[v + 0.295*yaw_rate, v - 0.295*yaw_rate]` (pursuit and
  pinger; `manual_keep_location` uses `yaw_rate/2`, halved 2026-10-27);
  `manualMove` treats `input[0]` as right; and the CLI's `move <left> <right> <s>` is stored
  as `[right, left]`.
- **Saturation is uniform everywhere, and it is one number.** PWM clamps to
  `[1100, 1900]`; thrust is bounded by a single rule at every point it can leave a node: `ThrustAllocator.allocate`, `master_control.publish_thrust`,
  `robot_interface.manualMove` and `simulation_interface` all scale the pair by **one factor**
  rather than clipping each side, and all four read a `thrust_limit` parameter defaulting to
  20.0 N. `thrust_limits.scale_to_limit` is the shared implementation.

  **Why uniform, not per-side.** The two thrusters do not carry independent signals — they
  carry one wrench, split into a common mode and a differential (`f = X/2 ± N/0.59`). Clipping
  each side on its own does not clamp that command, it *rewrites* it:

  | | right, left | surge X | yaw N |
  |---|---|---|---|
  | commanded | `[+45, +18]` | 63.0 N | 27.0 N |
  | old per-side clip | `[+20, +18]` | 38.0 N | **2.0 N** |
  | uniform scale | `[+20, +8]` | 28.0 N | 12.0 N |

  The old clip collapsed the differential from 27 N to 2 N, so the harder the controller asked
  for a turn the straighter the boat went — it diverged from the path instead of converging.
  Uniform scaling keeps the right:left ratio, so the direction of the wrench survives and the
  boat holds its commanded turn-per-metre and simply travels it more slowly.

  `publish_thrust` returns the saturated vector, so `/monitoring_data[7:9]`, the `.npy`
  `u1/u2` and the CSV's thrust columns agree with the wire. Thrust recorded above ±20 N in
  older logs comes from `solve_LoS`, which goes through no allocator (30–40 N, then
  per-side clipped), and from simulation, which applied no limit at all.
- **The 0–2 N band does not turn the propellers, and one law lives in it.** The bollard-pull
  table maps 0 N → PWM 1500, 1 N → 1514, 2 N → 1525, so the whole 0–2 N command range lands
  inside a T200 ESC's ~±25 µs neutral deadband. The table itself has no deadband — it crosses
  zero smoothly — so nothing in software knew. Per-law radius at which per-thruster force
  falls under 2 N:

  | law | channel | radius |
  |---|---|---|
  | `solve_LoS`, pinger, **real boat** (`k_v = 0.15`) | surge | **3.28 m** |
  | `solve_LoS`, pinger, simulation (`k_v = 2.0`) | surge | 0.25 m |
  | `solve_LoS`, manual target, real boat (double log) | surge | 0.30 m |
  | PID point-following | along-track | 1.33 m |
  | LoS station-keeping hold | surge | 0.70 m |
  | PID path-following | cross-track | 0.67 m |
  | LoS path-following | cross-track | 0.30 m |

  Only the first exceeds 2 m, so only it is floored. `solve_LoS` raises its **surge** term to
  `min_thrust * g * max(0, cos(bearing))`, where `g` ramps 0→1 over `hold_radius`. Both
  factors only ever *reduce* the floor, and both reuse a blend this file already applies
  elsewhere — `g` is the PID/LoS station-keeping fade, `max(0, cos)` is `los_guidance`'s own
  feedforward shaping. They are not decoration:

  * **`max(0, cos(bearing))`** — forward surge closes the range by `cos(bearing)` only. An
    ungated floor pushed the boat *away* from a target abeam or behind it while it turned
    round; the unfloored law does not, because its surge is ~0 there. The turn needs no help
    at those bearings — the differential is already 4.6 N per side at 90°, well clear of the
    deadband. Gating cut the modified region from every bearing to **|bearing| ≤ 69.5°**.
  * **`g`** — without it the commanded surge stepped by 1.64 N as the boat crossed 0.5 m
    inbound. With it the command is continuous in `d` (verified: the largest sample-to-sample
    jump scales linearly with sample spacing, slope `min_thrust/hold_radius` = 4 N/m).

  **What is provably not touched.** The `± 0.295*yaw_rate` differential — and therefore the
  yaw moment, which way the boat turns and how hard — is bit-identical across 865 200
  (range, bearing) grid points, and the floor never *lowers* the surge at any of them. The
  modified region is **0.61–3.27 m at |bearing| ≤ 69.5°, 5.7 % of the `d ≤ 12 m` plane**;
  everywhere else the output is bit-identical, and `min_thrust = 0.0` reproduces the original
  law exactly. The manual-target *pursuit* law never engages it (its own radius is 0.30 m,
  inside the fade-in) — but since 2026-09-03 the manual **keep-location** surge carries its
  own copy of the floor, written out separately because that one must not be coupled to
  `hold_radius`. See the keep-location entry below.

  **What it does change, stated plainly.** Inside that region the floor raises `X` while
  holding `N`, so the commanded surge/yaw ratio rises and the turn radius grows — at
  `d = 1 m`, `X/N` goes 2.68 → 7.66. That is a real change to the commanded wrench. It is
  defensible only because the unfloored command in that region *produces no motion at all*:
  a closed-loop check against an explicit 2 N-per-side deadband has the boat starting 0.8–3.2 m
  dead ahead of a pinger and **never moving**, and starting at 5–8 m it closes to 3.07 m and
  stalls there — the predicted boundary. With the floor it converges from every start, with no
  hunting (tail swing < 0.01 m).

  It settles at **~0.81 m**, not at `hold_radius`: the fade-in itself dips back under the 2 N
  breakaway around 0.8 m, so the boat parks there. That is the price of continuity over a hard
  step, and it is well inside the 2 m bar this work was measured against.
- **A reached manual target is held, not abandoned (2026-09-03).** `solve_LoS` used to latch on
  arrival — one second astern, then zero thrust for the rest of the run, cleared only by a new
  target. Any current then carried the boat out of the survey area with the controller
  commanding nothing, and on the real boat the latch never armed at all (`safety_distance` is
  −1.0 there), so the point law hunted instead of settling. `manual_keep_location` replaces the
  latch with a state re-evaluated every tick: inside `manual_hold_radius` the boat is on
  station, beyond `manual_reacquire_radius` the pursuit law takes back over, and in between the
  surge is proportional to the gap, capped, `max(0, cos(bearing))`-shaped and floored to
  `min_thrust`. **The yaw channel is untouched**, so the differential — which way the boat turns
  and how hard — is the law it always was; only the common-mode surge is replaced. The gains
  are chosen so the hold meets the pursuit law at the handover rather than stepping there:
  8.38 N against 8.00 N on the real boat, 15.42 against 15.00 in simulation.

  Measured on the harness plant with an explicit 2 N per-side deadband, starting on station,
  real gains, final distance after 400 s:

  | current | keep-location | old latch | old real-boat pursuit |
  |---|---|---|---|
  | 0 N | 0.00 m | 0.00 m | 0.00 m |
  | 2 N | 1.00 m | 27.2 m | 0.30 m |
  | 8 N | 1.50 m | 108.7 m | 0.69 m |
  | 12 N | 1.75 m | 163.0 m | 1.19 m |

  The pursuit column parks tighter but is not an alternative: approaching from 5 m in calm
  water it overshoots and runs away to 472 m on this model, the C8 failure mode, while the hold
  settles at 0.22 m. The floor is what makes 1.00 m reachable — unfloored the proportional term
  does not clear 2 N until 1.25 m, so the boat cannot reach the station it was told to hold.
  `manual_hold_radius <= 0` disables the hold and restores the previous law bit-identically.

  Two limits worth keeping in mind. The floor applies to the **common-mode surge**, not per
  side, so the inner thruster can still sit under 2 N — flooring per-side would alter the
  differential, the one thing this must not do, and a per-side deadband is not invertible
  without changing the wrench. And the modified region still reaches 3.27 m rather than 2 m;
  pulling that in means raising `point_k_v` (0.15 → 0.246 puts the law's own 2 N crossing at
  exactly 2.00 m), which is a change to the control law itself — it lifts far-field surge at
  50 m from 10.70 N to 12.94 N — and has deliberately **not** been made here.

## Bag replay — evidence (2026-10-27)

Run `2026_10_27-16_10_14-HarbourMPC`: roll std 6°, ±20° at 0.6–2 Hz, recorded at **4.2 Hz**.
The autopilot's ATTITUDE (#30) from `/uas1/mavlink_source`, timed by `time_boot_ms` (the
lower envelope of receive − boot removes WiFi delays up to 119 ms), matches
`/mavros/imu/data` to < 0.1°. On that bag's 25 Hz bursts decimated to 4 Hz, the Hermite
interpolation is 0.60° RMS / 1.5° max against slerp's 1.30 / 3.2. The heave gate pins the
waterline there (EKF z std 1.73 m, 37 % clipped: altitude noise). End to end
(`flat_5p0m`, `placement:=origin`): all 1201 pings used `pose_100hz` (age ≤ 20 ms), replayed
roll correlates with the bag's ATTITUDE at 0.9987 (0.9993 at `rate:=2`), and the two
transducer heights are antiphase, = 0.285·sin(roll), corr 1.000. Synthetic 60 s bag: pose
error 1 mm, attitude 0.01°, 0.8 mm drift over 2 s paused under `very_strong` current +
`rough` waves.
