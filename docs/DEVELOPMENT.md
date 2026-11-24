# BlueBoat-Control — development guide

How to work on this module: the AI coding assistant setup, the tests, the rules and what to
improve next. For using the module, see the [README](../README.md) and the
[user guide](USER_GUIDE.md).

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only this module:
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Control && claude
```

Starting from the superproject is better: Claude then also reads the project-wide rules, the
interface contract between modules and the shared skills (test commands, file formats).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: what the module is and its non-negotiable rules |
| `.claude/*.md` | Topic files: interface, build and run, control architecture, data and trajectories, log reviewer |
| `.claude/rules/*.md` | Load the right topic file automatically when Claude reads or edits a matching file |
| `.claude/TODO.md` | Open work |
| `blueboat_control/src/CLAUDE.md`, `blueboat_description/CLAUDE.md`, `log_reviewer/CLAUDE.md` | Folder notes, loaded when Claude works in that folder |

These files are written for Claude, but they are plain text and worth reading yourself before a
change: they record why things are the way they are.

## 3. What to set up after a fresh clone

Some files are deliberately not tracked by git (see `.gitignore`):

- **`.claude/settings.json`** holds one hook: after every edit, it runs the interface guard below.
  To recreate it:
  ```json
  {
    "hooks": {
      "PostToolUse": [
        {
          "matcher": "Edit|Write|MultiEdit",
          "hooks": [
            {
              "type": "command",
              "command": "sh -c 'S=\"$CLAUDE_PROJECT_DIR/.claude/tools/interface_inventory.py\"; [ -f \"$S\" ] || S=\"$CLAUDE_PROJECT_DIR/BlueBoat-Control/.claude/tools/interface_inventory.py\"; [ -f \"$S\" ] || exit 0; exec python3 \"$S\" --check \"$(dirname \"$S\")/interface_baseline.json\"'",
              "timeout": 20
            }
          ]
        }
      ]
    }
  }
  ```
- **`.claude/tools/`** holds the interface guard (`interface_inventory.py` and its
  `interface_baseline.json`). It compares every topic, service, parameter and message field with a
  saved baseline. It is not in git, so a fresh clone has no interface guard: ask the owner for the
  folder. Without it, the hook above does nothing.
- **The Python environment**: `~/ros2_ws/.venv` with acados, and `ACADOS_SOURCE_DIR` exported (README,
  steps 3 to 5). Two interpreters matter:
  - `~/ros2_ws/.venv/bin/python3` has acados, casadi and pandas. The control nodes need it.
  - `/usr/bin/python3` has ROS, numpy, scipy and matplotlib, but not acados.

## 4. Tests

Run them from `BlueBoat-Control/`, with the workspace sourced (`source ~/ros2_ws/env.sh`).

```bash
/usr/bin/python3 test/test_path_update.py         # live path update logic, no ROS
/usr/bin/python3 test/test_path_update_log.py     # path update logs and filing, no ROS
python3 test/test_bag_replay_track.py             # bag replay track (more checks with ROS and the recorded bag)
python3 test/test_yaw_wrap.py                     # heading at +-180 degrees, all controllers (needs the venv)
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py   # skips without a log under ~/ros2_ws/data
```

**The controller harness** proves that a change did not alter the controllers: it replays many
scenarios tick by tick and compares the old and the new code.

```bash
python3 test/mc_harness.py --compare HEAD --param gain_profile:=sim   # working tree against the last commit
python3 test/mc_harness.py --replanner                                # offline live path update scenarios
python3 test/gazebo_path_update.py --replanner --out-dir /tmp/gz      # in Gazebo, headless (slow)
```

Run the harness after any change to `master_control.py` or what it imports. There is no lint or
type checker.

**When running Gazebo tests**, never stop simulations with a machine-wide `pkill`: another
simulation may be running. Start each launch in its own process group and set `ROS_DOMAIN_ID`
and `GZ_PARTITION` per session.

## 5. Rules for changes

- **This module is frozen at version 1.0 and changes by additions only**, on the owner's explicit
  request. Nothing that works is renamed, retyped or changed in behaviour unless the harness shows
  it identical.
- **Never rename or retype a topic, service or parameter.** The Mission Control Station, the sonar
  modules, Belief and the Replanner depend on them. A change of QoS (delivery settings) is made at
  both ends or not at all.
- **Simulation and the real boat expose the identical ROS interface.**
- **`enable_motors` gates all thrust**, and a closed gate sends neutral. No new path may bypass it.
- **The reference moves with the boat's measured progress, never with the clock.**
- **The simulated boat model is frozen with the simulator.** Do not retune it.
- **Recorded runs are never modified.**
- AI sessions never commit, push or tag: the owner does.

The full list, with the reasons, is in `.claude/CLAUDE.md`.

## 6. Next steps

From `.claude/TODO.md`, in plain words:

1. **Test the live path update on water**, with PID first, following
   [PATH_UPDATE_REAL_WATER_TEST.md](PATH_UPDATE_REAL_WATER_TEST.md). So far it is shown in
   simulation only.
2. **A "Replanner mode" in the Mission Control Station**, so a replanner-driven run gets the
   station's safety buttons instead of being started from the command line.
3. **Find why the target stalls every 5 seconds** in Gazebo runs of designer paths: the logged
   target freezes for a few ticks, then catches up by 0.1 to 0.4 m.
4. **Give the real-boat MPC a model of the identified boat.** Its damping values are about 2.5 to
   3 times those identified from the field logs.
5. **Measure the real-boat MPC solve time in a field run** before using MPC with path updates on
   the boat.
