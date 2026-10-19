# TODO — BlueBoat-Control

Re-opened 2026-10-15 for the follow-ups of the **live path update**
(`docs/PATH_UPDATE_*.md`). It was ordered by the user, as was this file. Before that, the
module had no backlog since the v1.0 freeze.

## Live path update — open

- [ ] **First supervised on-water test**, `docs/PATH_UPDATE_REAL_WATER_TEST.md` (PID first).
  - Everything so far is demonstrated in simulation and in the deterministic harness only.
  - The tolerances (0.20 m, 10°) and the lead rule rest on single-site field data (P2) and on
    simulation.
- [ ] **MCS "Replanner mode"** (`docs/REQUESTS.md` R-1). Until then a replanner session is
  started from the CLI, without the station's safety UI.
- [ ] **BlueBoat-SSS-Sim `full_mission_launch.py`** should pass `path_source` (R-3).
  Built-world simulations cannot run replanner-owned runs yet.
- [ ] **Superproject root CLAUDE.md**: apply the rows and the D8 entry in `docs/REQUESTS.md`
  R-4. The user applies it; sessions do not edit the root interface tables.
- [ ] **Optional plan visualisation topic** (R-5), only if MCS/GCS ask. Nothing draws an
  updated path's geometry today. `/set_path` and bag replay show the run-start path.
- [ ] **Real-boat MPC with path updates.** Not to be tried until the real-boat MPC solve time
  has been measured in a field run (`src/CLAUDE.md` §4). The lead floor for real MPC is the
  untimed 2.5 s horizon.
- [ ] **Yaw-coherence check.** It catches compass/ENU and most deg/rad errors, not all (90 deg
  sent as radians passes). If the Replanner ever sends degrees by mistake this is a weak
  guard; a unit field in the request would fix it. That is a typed-interface change, so it
  needs a cross-repo decision.

- [ ] **`temp/`** holds a temporary quick check (`quick_mpc_path_change.py`, see
  `temp/README.md`; passed on MPC 2026-10-15). Delete it once it is no longer useful: the
  permanent tests are in `test/`.

## Housekeeping noticed on the way (not changed: outside the feature's scope)

- [ ] `blueboat_control/src/TRAJECTORY_SYSTEM.md` §6 states the MPC window as 2.5 s / 15
  poses. The code requests `mpc_horizon + 1` poses: 16 real, and 31 over 6.0 s in simulation.
- [ ] `yaml_trajectory.py` docstring points to `docs/08_trajectory_format.md` in the station
  repository. The file is `05_trajectory_format.md`.
- [ ] `simulation_interface.write_origin_sidecar` docstring says the sidecar waits for a GPS
  anchor. The code writes it on the first row, with lat/lon 0 if no anchor yet.
- [ ] Gazebo test runner lesson: a launch started as a shell background job (`cmd &`) ignores
  SIGINT, so it cannot shut down and file its run. `test/gazebo_path_update.py` runs each
  launch in its own process group instead.
