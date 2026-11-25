# TODO — BlueBoat-Control

Open follow-ups of the **live path update** (`docs/PATH_UPDATE_*.md`), ordered by the user.

## Live path update — open

- [ ] **First supervised on-water test**, `docs/PATH_UPDATE_REAL_WATER_TEST.md` (PID first).
  Everything so far is demonstrated in simulation and the deterministic harness only; the
  tolerances (0.20 m, 10°) and the lead rule rest on single-site field data and simulation.
- [ ] **MCS "Replanner mode"** (`docs/REQUESTS.md` R-1). Until then a replanner session is
  started from the CLI, without the station's safety UI.
- [x] **BlueBoat-SSS-Sim `full_mission_launch.py`** passes `path_source` (R-3, done
  2026-11-18 in the simulator's launch file; no Control change).
- [ ] **Optional plan visualisation topic** (R-5), only if MCS/GCS ask. Nothing draws an
  updated path's geometry today; `/set_path` and bag replay show the run-start path.
- [ ] **Real-boat MPC with path updates.** Not before the real-boat MPC solve time has been
  measured in a field run (`src/CLAUDE.md` §4). The lead floor for real MPC is the untimed
  2.5 s horizon.
- [ ] **Yaw-coherence check.** It catches compass/ENU and most deg/rad errors, not all (90°
  sent as radians passes). A unit field in the request would fix it — a typed-interface
  change, so a cross-repo decision.

## Identified plant and field-equivalent simulation (2026-11-13) — open

Done and what it rests on: `docs/PARAMETER_REGISTER.md`; requests to other modules:
`docs/REQUESTS.md` R-6 … R-10.

- **Thrust asymmetry, sway, κ and mass stay brackets** (register §2, §3; `docs/REQUESTS.md`
  R-10). No further field measurement is planned (owner, 2026-11-14): they are parameters of
  the sensitivity study, not open measurements.
- [ ] **The real MPC model is not the identified plant.** Left untouched by order. Its surge
  damping is ≈ 3× the identified secant at 0.5 m/s, its yaw damping ≈ 2.5×. A column that is
  the secant linearisation of the identified plant (`m − a_u` 35.7 kg, `d_u` ≈ 10–20 N·s/m by
  speed, `iz − a_r` 19.2 kg·m², `d_r` ≈ 13–22 N·m·s/rad by speed, in commanded units) was
  planned as an extra `gain_profile` value and not built.
- [ ] **Reference stalls every 5 s in Gazebo `from_yaml` runs.** The logged target freezes for
  a few ticks and catches up by 0.1–0.4 m, every ≈ 100 ticks; present in a log of 2026-11-12
  before this work, absent from the library shapes of 2026-11-10 and from the harness. Cause
  not found (path service latency? the file watch?).
- [ ] **The real PID does not hold a 1.0 m/s route** on the identified plant (falls 1.5–2.7 m
  behind its target, register §8); it was only ever flown at 0.5 m/s.
- [ ] **A stale MPC iterate after a manual excursion** is only moved by whole turns
  (`_align_iterate`); a full re-seed when the boat is far from it was not attempted.

## Notes

- **Never stop simulations with a machine-wide `pkill`** (2026-11-13): another session may be
  running Gazebo on its own ROS domain. Start each launch in its own process group (`setsid`)
  and signal that group only; set `ROS_DOMAIN_ID`, `GZ_PARTITION` and `ROS_HOME` per session.
- **`off` is a YAML boolean**: `odom_profile:=off` on a command line arrives as `false` and
  fails the parameter's type. The pass-through value is named `passthrough`.

- Gazebo test runner lesson: a launch started as a shell background job (`cmd &`) ignores
  SIGINT, so it cannot shut down and file its run. `test/gazebo_path_update.py` runs each
  launch in its own process group instead.
