# Live path update — report (2026-10-15)

## What was built

### The entry point

`/blueboat/update_path`, a new `blueboat_interfaces/srv/UpdatePath`. It is served by
`master_control` in every launch, simulation and real.

A **replanner-owned** run (`path_source=replanner`):
- starts holding at the boat, with the manual-target keep-location law;
- takes its first path through the service;
- then takes any number of replacements of "the rest of the path from change point c".

**Fixed-mission runs** (`path_source=mission`, the default) are unchanged and refuse
updates with `mission_owned`.

### The checks

Every request gets one answer: `ok`, or one of 17 machine-readable codes, each with a
human-readable message. The checks run in this order:
1. **State gate:** E-STOP / not ready, idle session, fixed mission or pinger, operator manual
   target or arrows.
2. **Request validity.**
3. **Change-point range.**
4. **Lead:** c ≥ τ + max(path_time + 2·dt, gov_Lmax/U_d).
5. **Continuity at c:** 0.20 m and 10°; the heading check is skipped where the reference is
   stationary.

A refusal changes nothing.

### Progress and traceability

- **Progress:** τ itself, the governor's path parameter, never a second notion. It is published
  on `/blueboat/path_progress` at 10 Hz with the state, the active and followed ids,
  `min_change_point`, the end of the path and the last answer.
- **Traceability:** `/blueboat/path_update_event` becomes the run's `-pathlog.csv`, using the
  poslog's three-row header rules. It is filed into the run folder and created only if the
  run received a request.

### Code

- **Core logic:** `_custom_libraries/path_update.py`, ROS-free; every decision is made there.
- **Wiring:** `master_control.py` section 7.
- **Pathlog:** `path_update_log.py`, plus small hooks in `robot_interface` and
  `simulation_interface`.
- **Filing:** one entry in `poslog_report`.
- **Launch files:** a `path_source` argument in both.

### Documents

- `PATH_UPDATE_DESIGN.md` (why, with sources);
- `PATH_UPDATE_INTERFACE.md` (the contract, incl. the MCS / Replanner integration guide);
- `REFERENCES.md`;
- `REQUESTS.md`;
- `PATH_UPDATE_REAL_WATER_TEST.md`.

## Demonstrated

| Claim | Evidence | Result |
|---|---|---|
| Fixed missions unchanged (N11) | `test/mc_harness.py --compare HEAD`: deterministic tick-level harness; PID/LoS/MPC × circle, kin_square, fsin, manual target, arrows, run session, plus real parameters | **21/21 bit-identical** (thrust, monitoring, τ). Sensitivity: a 1e-7 m change of `gov_Lmin` changes 1456/1600 ticks |
| Every answer code, junctions, determinism | `test/test_path_update.py` (ROS-free) | **102/102** |
| Pathlog format, lazy creation, filing | `test/test_path_update_log.py` | **24/24** |
| Closed loop, offline, all controllers | `test/mc_harness.py --replanner`: hold, first path, 5 updates (1 mid-turn, 2 back-to-back), refusals (offset, lead, manual, E-STOP, before first tick), end of path; session stop/new run, mission refusal | **3 × (19 + 11) checks pass** |
| Gazebo, all controllers | `test/gazebo_path_update.py --replanner`: the same sequence as the offline harness through a client written only from the interface doc; then checks on the **filed** run folder | **13/13 for PID, LoS and MPC.** Handling ≤ 1.36 ms; service round trip ≤ 2.9 ms; the target trail never jumps (max 0.14 m per 0.33 s log row = authored speed); pathlog rows as expected. Boat–target median: PID 0.89 m, LoS 1.16 m, MPC 0.04 m, the controllers' known trailing distances |
| Gazebo, MPC, simple scenario | `temp/quick_mpc_path_change.py` | **8/8**: incompatible path refused, compatible one followed at 0.04 m mean / 0.08 m max, junction step 0.020 m |
| Fixed mission in Gazebo before/after | `test/gazebo_path_update.py --regression`: kin_square, 146 s, a HEAD build vs the current build, plus the current build twice for Gazebo's own noise | **HEAD-vs-new below the noise floor on every controller.** Track RMS difference: PID 0.044 m (noise 0.078); LoS 0.001 m (noise 0.073); MPC 0.023 m (noise 0.058). Mean boat–target distance equal to 1 mm |
| Existing gates | interface guard (re-baselined: 13 additions + `UpdatePath`, 0 removals), `log_reviewer/smoke_test.py` | exit 0; 121/121 |
| Real-time | measured, design D-8 | handling ≤ 2.2 ms at the 20 000-sample cap (≤ 1 ms in every closed-loop run); per tick ≤ 3 ms incl. MPC; 50 ms budget |

## Assumed, not demonstrated

- **Behaviour on the real boat.** Nothing has been tested on water. The tolerances and the lead
  rest on single-site field statistics and on simulation. Protocol:
  `PATH_UPDATE_REAL_WATER_TEST.md`.
- **The real-boat MPC with updates.** Its 2.5 s horizon has never been timed in the field.
- **Literature.** R4 (Fossen ch. 12) was read from the author's lecture slides, not the book.
  R1 was read as its preprint.

## Open items

Tracked in `.claude/TODO.md`:
- the on-water test;
- MCS Replanner mode (usable today through the Launch dialog's Extra args);
- `full_mission_launch` support in BlueBoat-SSS-Sim;
- applying the root CLAUDE.md lines (`REQUESTS.md` R-4);
- an optional plan visualisation topic;
- the weak deg/rad guard;
- deleting `temp/`.
