# Live path update — first supervised on-water test

This is a short protocol for the **first live path updates** on the real boat. The operator
is ready to stop at all times.

**Prerequisites:** everything in §0 has passed.
**Contract:** [PATH_UPDATE_INTERFACE.md](PATH_UPDATE_INTERFACE.md).
**Commands and file locations:** all from `~/ros2_ws`.

Until MCS has a Replanner mode (`REQUESTS.md` R-1), a replanner session is started from the
command line. The station's safety controls are therefore replaced by the prepared terminals
of §2. Read §2 before going on the water.

## 0. Gate before the field (all must pass, same build as the one taken out)

```bash
cd ~/ros2_ws && colcon build --packages-select blueboat_interfaces blueboat_control
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source .venv/bin/activate
cd src/BlueBoat-SideScanSonar/BlueBoat-Control
python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json
/usr/bin/python3 test/test_path_update.py && /usr/bin/python3 test/test_path_update_log.py
python3 test/mc_harness.py --compare HEAD          # fixed missions bit-identical
python3 test/mc_harness.py --replanner --controllers PID
python3 test/gazebo_path_update.py --replanner --controllers PID --out-dir /tmp/gz
```

**Controller for the first test: PID.** It is field-proven (P2), trails its target by
0.7–1.2 m, and runs the 2-pose window. The real-boat MPC horizon (2.5 s) has never been timed
in the field. Do not combine its first field timing with the first path update.

## 1. Dock check (boat tied up, motors gate closed)

```bash
ros2 launch blueboat_control BlueBoat_launch.py session_mode:=true controller_type:=PID \
    enable_motors:=False path_source:=replanner note:=pu_dock
```

1. Configure and start a run:

   ```bash
   ros2 topic pub --once /blueboat/run_command std_msgs/msg/String \
     '{data: "{\"cmd\": \"configure\", \"controller_type\": \"PID\", \"use_pinger\": false, \"note\": \"pu_dock\", \"mission\": \"dock\", \"path_source\": \"replanner\", \"seq\": 1}"}'
   ros2 topic pub --once /blueboat/run_command std_msgs/msg/String '{data: "{\"cmd\": \"start\", \"seq\": 2}"}'
   ```
2. Progress must show `state: awaiting_path` with `hold`:
   `ros2 topic echo /blueboat/path_progress --full-length`.
3. Send a first path with the demo client's geometry (or by CLI, interface §10.6). Check:
   - `following`;
   - one `activated` event;
   - CSV `actuation_state` = 0, i.e. motors gated and only neutral is written.
4. Stop the run. Check:
   - `Robot_data/<run>/` holds the poslog, `-origin.yaml`, the PNG, the bag and
     **`-pathlog.csv`**;
   - **no** `-path.yaml` (a replanner run follows no file).

## 2. Safety set-up on the water

**Terminal A** holds the stop, typed but not sent:

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: stop"      # E-STOP: zero, disarm, latch
```

`enable` releases the E-STOP. It is **never** sent by a path update.

**Terminal B** holds the run stop:

```bash
ros2 topic pub --once /blueboat/run_command std_msgs/msg/String '{data: "{\"cmd\": \"stop\", \"seq\": 99}"}'
```

**Fallback for loss of the laptop:** the link guardian, configured as in
`link_guardian/README.md`, or the hand transmitter after `default`.

**Conditions:**
- an open area at least 40 m × 40 m clear of obstacles;
- wind and current low; log them;
- USBL not required.

**Abort immediately (Terminal A)** on any of:
- unexpected thrust;
- the boat leaving the planned area;
- > 3 m off the reference for > 10 s;
- a refusal that should not have happened;
- loss of progress messages.

## 3. Runs

Launch as in §1 with `enable_motors:=True note:=pu_water`. The bag is recorded by default.

Use `test/path_update_demo.py` as the client. It waits for the hold and then sends:
- the first path;
- a clean update;
- a mid-turn update;
- two back-to-back updates;
- one refused update;
- a final update;

and it waits for the end. Speeds are 0.4 m/s and turn radii 4–6 m. Lengths are about 30 m per
leg, so check that the area fits before starting.

| Run | What | Expected |
|---|---|---|
| W1 | start → hold 30 s → **first path only** (one straight line 15 m) → end → stop | holds within ~1 m; `activated P0`; follows; `end_reached`; holds the last pose |
| W2 | demo script, full sequence | §4 criteria at every junction |
| W3 | during following: E-STOP (A), send an update, `enable`, stop (B) | update refused `not_ready`; thrust zero at once; after `enable` following resumes on the same path; stop clears |
| W4 | during following: set a manual target (station or CLI), send an update, release with `[0, 0]` | refused `operator_override`; τ frozen; resumes the same path |

## 4. Pass / fail

All of these must hold; read them from the filed run folder.

| Criterion | Measure | Pass |
|---|---|---|
| Every request answered | client log / pathlog rows | 1 answer per request, expected code, round trip < 1 s |
| No reference jump at a junction | poslog `target_x/y` row-to-row step | ≤ U·0.33 s + 0.05 m (≈ 0.18 m at 0.4 m/s) |
| Tracking across a junction | cross-track to the active path, 0–20 s after each `activated` | back under 1 m within 15 s, never > 2 m |
| No thrust kick at activation | `.npy` u1/u2 in the 1 s around each activation | no saturation (\|u\| < 20 N); no step larger than the run's ordinary tick-to-tick variation |
| Refusal changes nothing | progress `active_id` / `t_end` before and after; target trail | identical; continuous |
| Safety precedence | W3, W4 | as in §3 |
| Traceability | `-pathlog.csv` in the run folder | rows for every request, activation, end, clear |

On failure: stop, file the run, and **do not** retry on the water. Reproduce offline from the
bag and the pathlog; the bag holds the event and progress topics (project_synthesis §8.6:
iteration happens offline).

## 5. What to log (beyond the automatic files)

**Per run:**
- time;
- wind and current (estimate);
- sea state;
- controller;
- the client command line;
- any manual intervention and its time.

**Automatic per run:** poslog, `-pathlog.csv`, PNG, bag, `.npy`.

**Copy** the client's `--out` JSON next to the run folder. It holds every answer and the sent
geometry.
