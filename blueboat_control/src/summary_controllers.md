# BlueBoat Controllers

Companion to [TRAJECTORY_SYSTEM.md](TRAJECTORY_SYSTEM.md). This document summarizes what each controller does with the target/path.

The package provides four controller modes:

| Controller | Main idea | Path following | Model / solver |
|---|---|---:|---|
| `MPC` | Predictive optimal control | Yes | Boat model + acados/CasADi |
| `PID` | Cascaded PID with LoS guidance | Yes | No |
| `LoS` | Kinematic line-of-sight control | Yes | No |

---

## 1. Line-of-sight guidance

`PID` and `LoS` use the same basic guidance law. Instead of aiming directly at the current path point, the controller aims at a point `Δ` metres ahead on the path.

```text
psi_d = gamma_p + atan2(-e_y, Delta)
```

- `gamma_p`: path heading
- `e_y`: cross-track error
- `Delta`: lookahead distance

A larger `Delta` gives smoother, less aggressive steering; a smaller `Delta` gives tighter tracking.

The implementation follows the standard lookahead-based marine LoS formulation. A reasonable starting point is roughly 2–4 vessel lengths; the shipped value is `2.5 m`.

---

## 2. `MPC` — Nonlinear Model Predictive Control

### Implementation

Every control cycle, MPC solves an optimisation problem over a finite horizon:

- predict the boat state using the 3-DOF model;
- follow future reference poses;
- penalise state error and thruster effort;
- enforce thruster limits inside the optimisation;
- apply only the first computed command, then solve again at the next cycle.

This is the only controller that explicitly uses the boat dynamics, including mass, added mass, damping and Coriolis effects.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `mpc_horizon` | `15` | Number of prediction steps |
| `mpc_time` | `2.5 s` | Prediction horizon duration |
| `Q_weight` | `diag(50, 50, 30, 1, 1, 1)` | Weight on `x`, `y`, `ψ`, `u`, `v`, `r` errors |
| `R_weight` | `diag(0.015, 0.015)` | Thruster-effort penalty |
| `input_bounds` | `±20 N` | Thruster constraints |

### Strengths

- Uses the vehicle model directly.
- Handles thruster saturation inside the optimisation.
- Can anticipate future path changes.
- Tuning is expressed mainly through weights and horizon rather than many PID gains.

### Weaknesses / implementation notes

- Performance depends strongly on model quality.
- Requires acados/CasADi and significantly more computation than the other controllers.
- Higher `R_weight` reduces aggressive control effort.
- Solver failure currently has no proper fallback; the implementation only reports the failure.
- Heading wrap-around must be handled carefully when comparing measured and reference yaw.

---

## 3. `PID` — Cascaded PID + LoS

### Implementation

The controller has two nested loops:

```text
Path / position error
        │
        ▼
Outer PID: position / heading
        │
        ├──► surge-speed reference
        └──► yaw-rate reference
                    │
                    ▼
Inner PID
        │
        ├──► surge force
        └──► yaw moment
                    │
                    ▼
             thruster allocation
```

LoS guidance provides the desired heading. The path's authored speed is also used as feedforward (`u_ff`), while the PID corrects the remaining error.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `outer_gains['x']` | `(3.0, 0.01, 0.0)` | Along-track error → surge reference |
| `outer_gains['psi']` | `(3.0, 0.01, 0.0)` | Heading error → yaw-rate reference |
| `inner_gains['u']` | `(1.0, 0.0, 0.0)` | Speed error → surge force |
| `inner_gains['r']` | `(1.5, 0.0, 0.0)` | Yaw-rate error → yaw moment |
| `pid_lookahead` | `2.5 m` | LoS lookahead distance |
| `thruster_limits` | `±20 N` | Applied after allocation |

### Strengths

- Does not require a detailed vehicle model.
- Very low computational cost.
- Separates position/heading control from force control.
- Easy to understand and maintain.
- Supports station keeping when configured for position holding.

### Weaknesses / implementation notes

- More parameters to tune than the simple LoS controller.
- Thruster limits are handled after the control law rather than inside it.
- Integral action has no anti-windup protection.
- The inner-loop gains should be tuned before the outer loops.
- Integral action can be useful for rejecting persistent disturbances.

A practical tuning order is: inner surge/yaw loops → outer heading loop → outer along-track loop → integral terms → lookahead distance.

---

## 4. `LoS` — Kinematic Line-of-Sight Controller

### Implementation

This is a lightweight controller built directly on the LoS desired heading:

```python
u_cmd = U_d * max(0, cos(psi_err))
X     = los_ku * (u_cmd - u)
N     = los_kpsi * psi_err - los_kd * r
```

The `cos(psi_err)` term reduces forward speed while the boat is strongly misaligned with the path.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `los_lookahead` | `2.5 m` | LoS lookahead distance |
| `los_ku` | `8.0` | Surge-speed error → force |
| `los_kpsi` | `10.0` | Heading error → yaw moment |
| `los_kd` | `1.0` | Yaw-rate damping |
| `los_speed_scale` | `1.0` | Multiplier on authored speed |

### Strengths

- Extremely simple and predictable.
- No vehicle model or solver required.
- Easy to debug.
- Useful as a lightweight baseline or fallback.

### Weaknesses / implementation notes

- No integral action, so persistent disturbances can produce a steady offset.
- Cannot perform station keeping as written: with `U_d = 0`, the surge command is zero.
- The shipped yaw response is relatively conservative.
- A pure proportional surge controller is limited by the vehicle's drag.

For disturbance rejection, Integral LoS (ILoS) is the standard extension: integrate a filtered cross-track error and include it in the LoS heading calculation.

A drag-feedforward term can also be added to the surge command:

```python
X = los_ku * (u_cmd - u) + drag_coefficient * u_cmd
```


## Main takeaway

`MPC` is the most complete controller because it predicts the boat dynamics and enforces actuator constraints during optimisation. `PID` is the practical classical alternative when a detailed model or solver is undesirable. `LoS` is the simplest path-following baseline. `Point-LoS` is a separate pure-pursuit controller intended for point targets rather than path tracking.
