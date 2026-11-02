# TODO — BlueBoat-Control

Open follow-ups of the **live path update** (`docs/PATH_UPDATE_*.md`), ordered by the user.

## Live path update — open

- [ ] **First supervised on-water test**, `docs/PATH_UPDATE_REAL_WATER_TEST.md` (PID first).
  Everything so far is demonstrated in simulation and the deterministic harness only; the
  tolerances (0.20 m, 10°) and the lead rule rest on single-site field data and simulation.
- [ ] **MCS "Replanner mode"** (`docs/REQUESTS.md` R-1). Until then a replanner session is
  started from the CLI, without the station's safety UI.
- [ ] **BlueBoat-SSS-Sim `full_mission_launch.py`** should pass `path_source` (R-3).
  Built-world simulations cannot run replanner-owned runs yet.
- [ ] **Optional plan visualisation topic** (R-5), only if MCS/GCS ask. Nothing draws an
  updated path's geometry today; `/set_path` and bag replay show the run-start path.
- [ ] **Real-boat MPC with path updates.** Not before the real-boat MPC solve time has been
  measured in a field run (`src/CLAUDE.md` §4). The lead floor for real MPC is the untimed
  2.5 s horizon.
- [ ] **Yaw-coherence check.** It catches compass/ENU and most deg/rad errors, not all (90°
  sent as radians passes). A unit field in the request would fix it — a typed-interface
  change, so a cross-repo decision.

## Notes

- Gazebo test runner lesson: a launch started as a shell background job (`cmd &`) ignores
  SIGINT, so it cannot shut down and file its run. `test/gazebo_path_update.py` runs each
  launch in its own process group instead.
