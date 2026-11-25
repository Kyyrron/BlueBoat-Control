# Pinger — field calibration and readiness checklist (2026-11-24)

**Why this is mandatory.** No recorded run had a still pinger: the brush robot always moved
(`docs/PINGER_FILTER_FINDINGS.md`). The filter's **latency (1.5 s), y sign (+1), mount yaw (0°) and
lever arm (0, 0) are provisional**. Its noise defaults (bearing 12°, range noise 20 % of range) come
from a 0.85 m deep pool. One 10-minute session with the pinger at a known point fixes all of them.

## 0. Before leaving (laptop, 5 min)

- [ ] Gates pass:
  ```bash
  /usr/bin/python3 test/test_pinger_filter.py
  python3 test/mc_harness.py --pinger-modes
  python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json
  ```
- [ ] `colcon build` done and sourced, so `pinger_estimator.py` appears in `ros2 pkg executables blueboat_control`.
- [ ] A GPS-surveyed point for the pinger: a pontoon corner, a buoy on a short line, a weight on a
  surveyed spot. Write down its latitude and longitude.

## 1. On site, Water Linked check (5 min, WL web UI at `http://192.168.2.94`)

- [ ] Note the model, the **locator type** (U1 battery with a depth sensor / A1 or D1 wired) and the
  firmware. After launch, `/uw_gps/info` records what the API exposes:
  `ros2 topic echo --once /uw_gps/info --qos-durability transient_local --qos-reliability reliable`.
- [ ] **Depth source.** In 2026-10 the locator z read a near-constant 0.19 m. If the locator has no
  depth sensor, the unit needs the depth from the vehicle (`/api/v1/external/depth`). Find out which
  applies, and note it.
- [ ] **Receiver layout:** the 4 receivers sit at the boat's corners. Note the positions entered in the
  WL UI and their reference point. That point, relative to the BlueBoat's GPS/odom point, is the
  lever arm (`lever_x_m` forward, `lever_y_m` left).
- [ ] Receivers submerged and away from the thrusters' wash, as far as the mounting allows.

## 2. Calibration run (10 min) — the pinger STILL at the surveyed point, depth 1–3 m

Launch with the pinger logged and nothing steering to it. The station: Pinger "Log pinger only",
a path or "(no path)" with the arrow keys. From the command line:

```bash
ros2 launch blueboat_control BlueBoat_launch.py controller_type:=PID session_mode:=true pinger_mode:=log enable_motors:=True
```

Drive by hand (arrow keys) or with a short path:

1. [ ] **Slow 360° turns** at about 5 m, 10 m and 20 m from the pinger: 2 full turns at each radius,
   yaw rate about 0.2 rad/s. This identifies the latency and the mount yaw.
2. [ ] **Straight passes** past the pinger: 2 legs each way at 0.5 m/s, passing 5–10 m abeam. This
   identifies the latency along travel and the y sign.
3. [ ] **30 s stationary** at about 10 m. This gives the noise level.

Stop the run. The run folder holds the bag and the `-pingerlog.csv`.

## 3. Re-fit (laptop, 10 min)

```bash
CLI=blueboat_control/src/_custom_libraries/pinger_filter_replay.py
RUN=~/ros2_ws/data/Robot_data/<run folder>
# 1) latency, y sign, mount yaw: the combination that makes the still pinger stillest
python3 $CLI grid $RUN/*-bag --grid latency_s=0:3:0.1 y_sign=-1,1 mount_yaw_deg=-15:15:1 --out /tmp/cal
# 2) with the best values, the noise and the decisions
python3 $CLI run $RUN/*-bag --set latency_s=<best> y_sign=<best> mount_yaw_deg=<best> --out /tmp/cal_run
```

Re-fit, in this order:

| Parameter | How to read it |
|---|---|
| `latency_s`, `y_sign`, `mount_yaw_deg` | The `grid` best combination (lowest `local_scatter_median`). Cross-check the filtered track's distance to the surveyed point in `/tmp/cal_run/tracks.png` |
| `lever_x_m`, `lever_y_m` | From the WL receiver layout's reference point (section 1). If the filtered point still sits off the surveyed point by a constant amount that turns with the boat, adjust these |
| `sigma_bearing_deg`, `range_k` | Mean NIS on accepted fixes in `metrics.json` between 1 and 3. Above 3: increase them. Below 1: decrease them |
| `static_target` preset | If the target will be still (a pinger on a fixed object), use it |

Save the set (station: Advanced… → Save YAML, or `pinger_params.write_param_yaml`). Then:
- [ ] update the provisional values in `blueboat_control/config/pinger_filter.yaml`;
- [ ] update `docs/PARAMETER_REGISTER.md` §9, with the run's name as the evidence.

## 4. Follow test (5 min)

- [ ] Pinger still at the surveyed point. Station: Pinger "Follow pinger (no path)", controller
  **PID**, Start. The boat must approach and hold near the pinger.
- [ ] Repeat with **LoS** (its pinger law parks about 0.8 m away on the real boat: the `min_thrust` floor).
- [ ] Pull the pinger out of the water (or block it) for 10 s. The PINGER panel goes coasting, then
  stale. The boat **holds its position** (`/rosout`: "Pinger following paused"). It resumes when the
  status is tracking again.
- [ ] The station greys out MPC for Follow; a `configure` with MPC + follow is refused.

## 5. Log-mode tests (5 min)

- [ ] **With a path:** Pinger "Log pinger only", a short **non-loop** path, **MPC**. The boat follows the
  path and never turns toward the pinger. At the end it holds the final pose: the panel shows "Path
  complete — holding final pose", and the boat stays at the end until STOP RUN.
- [ ] **Without a path:** "Log pinger only" + "(no path)". After START the boat does not move.
  `/rosout` shows no `/path_request` warning. A manual target is held; the arrow keys drive.
- [ ] After each run: the run folder holds `-pingerlog.csv` and `-pinger-smoothed.csv`, and the
  report PNG draws the legacy, filtered and smoothed pinger.

## 6. If something is off

`FIELD_TUNING.md` → "Pinger filter" (symptom → knob). Every run can be re-filtered offline with a
changed set (`pinger_filter_replay.py run`) before the next one. Never edit a recorded file.
