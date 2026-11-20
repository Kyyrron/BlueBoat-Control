# CLAUDE.md — `blueboat_control/src` (controller and thrust-path evidence)

Loaded when working under `blueboat_control/src/`. `BlueBoat-Control/.claude/CLAUDE.md`
keeps the rules (N1–N11) and the contract; this file keeps the measurements, derivations and
tables behind its §4–§5. Section references (§n, Nn) point to that file.

On 2026-11-18 the detail moved **verbatim** to `BlueBoat-Control/.claude/src-control-detail.md` (§4) and
`BlueBoat-Control/.claude/src-thrust-detail.md` (§5 and the bag-replay evidence). **Read the relevant one
before changing a controller, the governor, the acados build, the thrust table or the RC-override path.**

## §4 Control architecture — what the detail file holds (`.claude/src-control-detail.md`)

- The controller object is built in `__init__`, not on the first tick (acados codegen froze the executor).
- Rebuilding between runs without stopping the loop; arrow keys and the dead-man.
- The generated acados solver is cached outside the working directory, and what "a real change" covers
  (verified 2026-09-04 against `acados_template` 0.5.1).
- A failed acados solve is not a command (C6).
- Reference generation (the governor: progress, never wall-clock time, N8), guidance, which law runs where.
- Zero authored speed: the station-keeping hold.
- Tuning knobs, all `declare_parameter`'d, and which values are derived.

## §5 Thrust path — what the detail file holds (`.claude/src-thrust-detail.md`)

- `/thruster_input` carries **`[right, left]`**, consistently across every code path (the allocation matrix,
  the URDF thruster order, `simulation_interface`, `solve_LoS`, `manualMove`, the CLI `move`).
- Saturation is uniform everywhere, and it is one number (the PWM clamp).
- The 0–2 N band does not turn the propellers, and one law lives in it (the bollard-pull dead band).
- A reached manual target is held, not abandoned (2026-09-03, `solve_LoS`).
- The bag-replay evidence of 2026-10-27: ATTITUDE timing against `/mavros/imu/data`, Hermite interpolation,
  the heave gate.

Simulated results follow `../SIM_VALIDITY_CONDITIONS.md` (superproject root); the identified plant and
`sim_config:=field_equivalent` are frozen with the simulator (no retuning).
