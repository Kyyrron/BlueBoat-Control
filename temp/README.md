# temp/ — quick live-path-update check (temporary, delete when no longer useful)

`quick_mpc_path_change.py` runs one Gazebo simulation the way the Mission Control Station
launches an empty-world simulation:

```
Sim_launch.py robot_file:=thrusters_ur trajectory:=station_keeping controller_type:=MPC note:=… path_source:=replanner
```

`path_source:=replanner` is what typing it in MCS's Launch dialog **Extra args** field adds.

The script then drives the run:

| Step | What | Expected |
|---|---|---|
| 1 | Wait while the boat holds at its spawn pose | progress state `awaiting_path` |
| 2 | **Path A**, first path: 30 m straight ahead at 0.4 m/s, followed about 10 s | accepted |
| 3 | **Path C**, *incompatible*: 1.5 m to the side of A at the change point, turned 30° | **refused** (`discontinuous_position`); A keeps being followed |
| 4 | **Path B**, *compatible*: continues A at the change point, left 90° turn (r = 5 m) then 10 m | accepted; becomes active when τ reaches the change point |
| 5 | Watch the boat 30 s on B, stop the simulation | the run is filed with its `-pathlog.csv`; PASS/FAIL per check |

## Run it

From `BlueBoat-Control/`, in a fresh terminal. **The venv is required**: `master_control`
and `simulation_interface` import `casadi` / `acados_template`, which only
`~/ros2_ws/.venv` has. Without it both nodes crash at launch and the boat never moves.

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/.venv/bin/activate        # or, in one line from ~/ros2_ws: source env.sh
python3 temp/quick_mpc_path_change.py           # headless, ~1.5 min
python3 temp/quick_mpc_path_change.py --gui     # with Gazebo and RViz; stays open until Enter
python3 temp/quick_mpc_path_change.py --controller PID
```

**What you see with `--gui`.** RViz opens with `temp/quick_path_change.rviz`: Control's
usual view plus two paths.

| In RViz | Meaning |
|---|---|
| **green** | the path Control is following: A, then A up to the change point + B once B is accepted |
| **red** | the refused path C |
| blue *Set Path* | the fixed-mission display (`/set_path`); in a replanner run it is only a dot at the origin, by design |
| moving arrow | the controller's current target on the followed path |

After the checks the simulation keeps running; press **Enter** to stop it.

**If something goes wrong.**
- **The venv is not active.** The script stops before launching and tells you to activate
  it.
- **A launched node crashes.** The script stops within a second and prints the error lines,
  instead of waiting.
- **Ctrl-C.** The first one stops the test; further Ctrl-C are ignored until the simulation
  is down and the run is filed (~10 s). This keeps an interrupted run from leaving an
  orphaned simulation behind.

**Before you start:** no other simulation or control node may be running. The script refuses
to start if one is.

**Where the output goes:** the run is filed under `/tmp/quick_path_change/data/Robot_data/`.
Use `--data-dir` to put it elsewhere.

## Result (2026-10-15, MPC, headless; re-run after the fixes below, same result)

All 8 checks passed:

```
A-straight     -> accepted=True  reason=ok
C-incompatible -> accepted=False reason=discontinuous_position   (1.500 m from path A at t=18.5; limit 0.200 m)
B-compatible   -> accepted=True  reason=ok                        (replaces the path from t=18.5, 8.45 s ahead of tau)
Boat distance to path B over 30 s: mean 0.04 m, max 0.08 m; largest reference step at the junction 0.020 m
```

The 0.020 m reference step is one 50 ms tick at 0.4 m/s, so there is no jump.

The pathlog rows, in order:

| Event | id | change point / reason |
|---|---|---|
| `cleared` | — | launch |
| `request_accepted` | A | — |
| `activated` | A | — |
| `request_refused` | C | `discontinuous_position` |
| `request_accepted` | B | c = 18.5 |
| `activated` | B | at τ = 18.5 |

## Fuller tests

These are the permanent tests; this folder is only the quick version.

| Script | What it runs |
|---|---|
| `test/gazebo_path_update.py --replanner` | first path, 5 updates (one mid-turn, two back-to-back), one refusal, end of path; all three controllers |
| `test/mc_harness.py` | the deterministic offline harness |

The contract is `docs/PATH_UPDATE_INTERFACE.md`.

## Fixes after the first user run (2026-10-15)

The first `--gui` run by the user showed a boat that never moved and no path in RViz.

**Cause:** the venv was not active, so `master_control` and `simulation_interface` died
with `ModuleNotFoundError` (`acados_template`, `casadi`) and nothing drove the boat. The
script then waited silently.

**Fixed in this folder:**
- an environment check before launching;
- crash detection with the error lines;
- the green / red paths in RViz;
- Enter-to-stop in `--gui`;
- Ctrl-C-safe shutdown.

**Fixed in `test/gazebo_path_update.py`:** the "is a simulation already running?" check now
matches executables, not any command line that merely mentions them. Before, a shell or a
`grep` containing `rviz2` was enough to block a run.

Nothing in Control's nodes changed.
