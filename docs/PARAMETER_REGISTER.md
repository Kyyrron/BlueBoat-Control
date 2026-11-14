# Parameter register — BlueBoat-Control

Every result-affecting number of the simulated plant, of its actuator and timing models, and of the
controller columns: value, unit, origin, interval or bracket, where it lives. Written 2026-11-13/14
(session 04 of the simulator completion round, acceptance test I6).

**Origin tags.** MEASURED = read from our own field logs. IDENTIFIED = fitted on FIT field logs (a kind of
MEASURED; the interval is a 95 % bootstrap interval over segments). VENDOR = the manufacturer's published
figure (URL in `REFERENCES.md` §V). DERIVED = computed from other entries by a stated rule. ASSUMED = neither,
always with a bracket. LEGACY = carried over unchanged from the BlueROV2 table, owned elsewhere (see §4).

**Evidence.** Scripts and outputs are outside the repository, under
`~/ros2_ws/sim_runs/sim_acceptance/04_control/` (`ident/`, `timing/`, `runs/`, `r4/`, `gates/`); the file named
in each row is relative to that folder. Data split: `~/ros2_ws/sim_runs/sim_acceptance/split_register.yaml`,
section `motion:`.

**Markers.** [VERIFIED] read in code or run in this session; [DOC] taken from a document; [UNCERTAIN].

---

## 1. What was identified, and on what

| | |
|---|---|
| Primary data | `Newton_vs_Speed`: pool run `2026_10_27-10_33_16`, 114 s, thrust stepped by hand through the arrow keys (symmetric 4/4, 10/10, 15/15, 20/20 N; coasts; differential ±7 and ±10 N at rest; 20/7.3 N under way). Odometry 4.0 Hz, thrust log 20 Hz. The raw CSV is no longer on disk; the processed export was checked against the independent controller log (thrust mean difference 0.12 N, pose median difference 0.000 m). `ident/newton_look.py` [VERIFIED] |
| Cross-check | The six runs of 2026-09-07 named FIT (sine weave and expanding square; PID, LoS, MPC), closed loop, wind 14–27 km/h [DOC]. `ident/surge_crosscheck.py`, `ident/yaw_crosscheck.py` |
| Units | **Commanded Newtons.** A commanded Newton is the vendor's bollard thrust of the M200 motor + weedless propeller at 16 V (§5), not a measured force. Every identified force coefficient scales with any error of that map; speed against commanded thrust does not |
| Model | 3-DOF surge / sway / yaw as Gazebo 8.15 integrates it (§3), plus an actuator model (§2) |
| What the sampling allows | 4 Hz position: surge drag, surge inertia, yaw gain and time constants ≥ 0.5 s. Not resolvable: ESC spin-up lag, the deadband edge, sway (no lateral excitation, sway speed ≤ 0.1 m/s against 0.05–0.1 m/s of noise), a right/left thrust asymmetry |

### 1.1 Results in commanded units (what the data say, before any split)

| Quantity | Value | 95 % interval | Source |
|---|---|---|---|
| Thrust for 1.0 m/s, both thrusters together | 20.2 N | [18.6, 23.7] | `ident/bootstrap.json` |
| Steady speed at 2 × 4 N / 2 × 10 N / 2 × 20 N | 0.63 / 0.99 / 1.41 m/s | [0.58, 0.66] / [0.92, 1.04] / [1.30, 1.47] | same |
| Surge inertia per commanded Newton, `m − X_u̇` | 35.7 kg | [22.2, 54.5] | same |
| Hull drag, quadratic | 8.58 N·s²/m² | [2.6, 19.7] | same |
| Propeller speed loss `kp` (per thruster) | 1.83 √N·s/m | [0, 2.98] | same |
| Coast decay `X_u\|u\| / (m − X_u̇)` (free of the thrust map) | 0.240 1/m | [0.12, 0.48] | same |
| Linear surge drag | 0 | fits to 0.00 when free | `ident/surge_fit2.py` |
| Fit residual, surge | 0.075 m on the along-track distance (0.079 m without `kp`; ΔAIC 18 for `kp`) | — | `ident/surge_fit2_scan.json` |
| Yaw: time constant at rest | 1.50 s | [1.30, 2.99] | `ident/yaw_fit5.json` |
| Fit residual, yaw rate | 4.5 °/s (0.079 rad/s) | — | same |

Drag and `kp` are poorly separated one by one and tightly determined together: the three derived rows at the top
are what the plant is held to. The plain quadratic-drag model without `kp` (58.9 kg [40.6, 74.7], 18.7 N·s²/m²
[17.5, 23.4]) gives the same steady speeds within 4 %; it cannot give both the measured acceleration from rest
(≈ 1.0 m/s² at 40 N) and the measured coast decay, which is why `kp` is kept.

**Against the BlueROV2 plant** (Gazebo, same thrust sequence, `ident/gz_compare.json`) [VERIFIED]:

| | Field | Identified plant | BlueROV2 plant |
|---|---|---|---|
| Speed at 8 / 20 / 30 / 40 N | 0.62 / 1.01 / 1.14 / 1.40 m/s | 0.61 / 0.99 / 1.22 / 1.40 | 0.24 / 0.48 / 0.64 / 0.78 |
| Thrust at 1 m/s | 20 N | 20 N | 59 N (cannot reach 1 m/s) |
| Yaw rate, ±7 N at rest | 17.6 °/s | 18.0 | 15.9 |
| Yaw rate, 20 / 7.3 N at ≈ 1.1 m/s | 3.0 °/s | 1.3 | 14.0 |
| Whole run, surge rms / correlation | — | 0.24 m/s / 0.92 | 0.46 m/s / 0.75 |
| Whole run, yaw-rate rms / correlation | — | 4.5 °/s / 0.86 | 8.0 °/s / 0.58 |

---

## 2. Actuator model (applied by `simulation_interface`, stated by the description)

`applied = thrust_gain_side · ( T − prop_speed_loss · √|T| · surge speed )`, per thruster, after the uniform
saturation. Read from the robot description at start-up; a non-negative node parameter overrides it.

| Parameter | Value | Unit | Origin | Interval / bracket | Where |
|---|---|---|---|---|---|
| `thrust_gain_right`, `thrust_gain_left` (κ) | 0.493 | — | DERIVED: (rigid mass 16.01 + surge added mass 1.60) / 35.7 kg identified | bracket 0.45 – 0.85 (lower: a 16 kg boat; upper: the vendor's own static thrust, 8.2 kgf for the boat against 2 × 5.63 kgf on the chart = 0.73, and a 14 V battery) | `blueboat_description/urdf/hydrodynamics_identified.xacro:74-75` |
| right/left asymmetry `a` | 0 | — | NOT IDENTIFIABLE: three model forms give −0.24, −0.12, +0.07 (`ident/yaw_fit3.json`, `yaw_fit_scan.json`, `yaw_fit4.json`) | bracket −0.25 … +0.10. The 2026-09-07 FIT runs needed +2.2 to +3.8 N more on the right (in wind); a refit of `a` on them gives −0.19 to −0.68, a constant torque of −0.13 to −0.31 N·m fits as well (`ident/yaw_crosscheck.json`) | same file (give the two gains different values), or parameter `thrust_gain` |
| `prop_speed_loss` (kp) | 1.83 | √N·s/m | IDENTIFIED | [0, 2.98]. Independent support: propeller pitch 14° at 75 % of 112 mm (66 mm) and the vendor's RPM chart give a zero-thrust speed of ≈ 1.3–1.6 m/s at 10 N, i.e. kp ≈ 2.0–2.4 [VENDOR + DERIVED] | `…/hydrodynamics_identified.xacro:76` |
| `esc_deadband` | 0 | N | NOT IDENTIFIABLE (a 1–2 N deadband moves the FIT speed predictions by ≤ 3 points, `ident/deadband_check.json`) | vendor: ±25 µs for the 30 A Basic ESC, nothing published for the ESC 500 the boat carries; the chart shows 2.1 N already at +26 µs | `blueboat_control/src/simulation_interface.py:129` |
| `thrust_lag_tau` | 0 | s | NOT IDENTIFIABLE at 4 Hz | a dead time of 0–0.3 s moves the inertia by < 4 % | `…/simulation_interface.py:130` |

Legacy plant: no actuator model (gain 1, loss 0): a commanded Newton is an applied Newton.

---

## 3. Hull, plant `identified` (`blueboat_description/urdf/hydrodynamics_identified.xacro`, `blueboat.xacro`)

Physical units for the rigid mass of `blueboat.xacro`; identified values are the commanded-unit results × κ.
Equations as integrated (gz-sim 8.15 `Hydrodynamics` + rigid body; the plugin's added-mass coupling has the
opposite sign to Fossen's in the horizontal plane — read in its source, `inventory/downloads/gz_sim8_hydrodynamics/`):

```
(m+A11) u' =  (m−A22) v r + X − Xuu u|u|
(m+A22) v' = −(m−A11) u r − Yvv v|v|
 Izz    r' = +(A22−A11) u v + ℓ (F_R − F_L) − Nr r − Nur |u| r
```

| Coefficient | Tag | Value | Unit | Origin | Interval / bracket | Where |
|---|---|---|---|---|---|---|
| Rigid mass `m` | `mass` | 16.01 | kg | LEGACY (the simulator's hull wrench mirrors it) | vendor: 14.5 kg bare + 2.4 kg per two battery packs + 0.3 kg per sonar head = 17.5 kg (2 packs) to 19.8 kg (4 packs), before mounts and cables; survey loading never weighed → 16 – 20 kg | `blueboat.xacro:36` |
| Surge added mass `A11` | `xDotU` | −1.60 | kg | ASSUMED: 10 % of the rigid mass (slender hull) | 5 – 15 %; with κ it reproduces the identified 35.7 kg per commanded N | `hydrodynamics_identified.xacro:48` |
| Surge drag, linear | `xU` | 0.0 | N·s/m | IDENTIFIED (fits to zero) | — | `:56` |
| Surge drag, quadratic `Xuu` | `xUabsU` | −4.23 | N·s²/m² | IDENTIFIED: κ × 8.58 | κ × [2.6, 19.7]; tied to `kp` (§1.1) | `:64` |
| Sway added mass `A22` | `yDotV` | −14.0 | kg | DERIVED: strip theory, two demihulls 1.2 m long, draft 0.06 m | 8 – 30 kg (draft 0.045 – 0.10 m) | `:49` |
| Sway drag, linear | `yV` | 0.0 | N·s/m | ASSUMED (no lift term) | — | `:57` |
| Sway drag, quadratic `Yvv` | `yVabsV` | −108.0 | N·s²/m² | DERIVED: cross-flow, C_d 1.5 on 0.144 m² | 54 – 240 | `:65` |
| Yaw inertia, total `Izz` | `iyy` + `nDotR` | 7.50 + 1.98 = 9.48 | kg·m² | IDENTIFIED (total). The split is forced: a rigid tensor needs `iyy ≤ ixx + izz = 8.06` | [8.11, 10.95] | `blueboat.xacro:74`, `hydrodynamics_identified.xacro:53` |
| Yaw damping at rest `Nr` | `nR` | −6.31 | N·m·s/rad | IDENTIFIED | [3.08, 7.10] | `:61` |
| Yaw damping, quadratic | `nRabsR` | 0.0 | N·m·s² | IDENTIFIED (fits to zero) | — | `:69` |
| Yaw damping per m/s of surge `Nur` | `nRabsU` | −4.76 | N·m·s/rad per m/s | IDENTIFIED, together with the sway coupling above | [1.36, 15.1]; 3.9 – 5.7 across the sway bracket | `:70` |
| Thruster lever arm ℓ | joint origins | 0.295 | m | VENDOR-consistent: motor axes 589 mm apart on the vendor CAD (±10 mm) | — | `thrusters_ur.xacro` |

**Yaw inertia from a mass distribution — not possible.** The vendor publishes no component masses and its
CAD (STEP/STL) carries no density, so the rigid yaw inertia cannot be computed from documents. What exists:
the 3.92 kg·m² Gazebo used (the `iyy` of a third-party model file; `blueboat.xacro:61`), a box estimate of
3.0 kg·m², and the identified total 9.48 kg·m² per κ. The split rigid / added is therefore not known; only the
total enters the yaw response.

**Real MPC column** (`master_control.py:500`, unchanged by order): `m − a_u` = 42.8 kg and `I_z − a_r` =
27.4 kg·m² in commanded units — close to the identified 35.7 kg and 9.48 / 0.493 = 19.2 kg·m², which suggests an
earlier identification of the same kind. Its dampings (`d_u` 29.3 N·s/m, `d_r` 44.7 N·m·s/rad) are higher than the
identified secants at 0.5 m/s (≈ 10 N·s/m; ≈ 18 N·m·s/rad in commanded units). Provenance still unknown.

---

## 4. Not this module's to set (kept numerically, change request to the simulator)

| Tags | Value | Why kept |
|---|---|---|
| `zDotW`, `kDotP`, `mDotQ` | −14.57, −0.12, −0.12 | BlueROV2 values. `BlueBoat-SSS-Sim` `config/sea_states.yaml` (`hull.damping`) and `sea/motion.py` mirror them by hand, and the simulator's heave / roll / pitch damping is sized as the complement of `zW`, `kP`, `mQ` |
| `zW`, `kP`, `mQ` | −17.955, −10.888, −20.761 | same |
| `zWabsW`, `kPabsP`, `mQabsQ` | −73.37135, −40.0, −40.0 | same |
| `ixx`, `izz`, mass, buoyancy box | 2.42369, 5.6403125, 16.01 kg, 1.2 × 0.91 × 0.376 m | the simulator's smoke test reads the first `<inertial>` block and subtracts the box buoyancy with these numbers |

Ownership of the degrees of freedom: surge, sway, yaw — this module (above). Heave, roll, pitch — the
simulator's hull wrench (hydrostatics and damping) on top of the nine values of this table. The simulator adds no
surge, sway or yaw damping (read in `sea/hull.py`, `docs/sea_state_model.md`), so nothing is counted twice there.

---

## 5. Thrust map (real boat, `custom_functions.generate_interpolator`, `custom_functions.py:92`)

| | |
|---|---|
| What it is | 33 points, PWM 1100–1900 µs against thrust; PCHIP. **It is the vendor's chart of the M200 motor with the weedless propeller at 16 V**, bollard: 32 of 33 values identical to `m200weedless_force_plot.js` (1110 µs: vendor 2.79 kgf, table 2.78). The BlueBoat does not carry T200 thrusters: 2 × M200 + 112 mm weedless propeller on Basic ESC 500 [VENDOR] |
| What it is not | A measurement on this boat, a figure at the battery's voltage (4S: 16.8 V full, 14.8 V nominal, 12 V minimum), or a thrust under way |
| Voltage | The weedless chart exists at 16 V only. For the same motor with the T200 propeller, thrust at a fixed PWM is 18–35 % higher at 16.8 V than at 14 V in the 2–20 N range [VENDOR, proxy] |
| Propagation | A scale error ε of the map multiplies every identified force and inertia in commanded units by (1 + ε) and leaves speed against command, time constants, and thrust per metre in commanded Newtons unchanged. It moves κ, hence only the response to external forces (waves, current, wind) and any figure in physical Newtons or Watts |

---

## 6. Loop timing

| Parameter | Field-equivalent | Legacy | Origin | Where |
|---|---|---|---|---|
| Control loop | 20 Hz | 20 Hz | — | `master_control.py:428` |
| Thrust applied (`thrust_rate_hz`) | 20 Hz | 10 Hz | field: `robot_interface` streams RC override at 20 Hz; 20.0 Hz in every 2026-10-27/28 bag [MEASURED] | `simulation_interface.py:102` |
| Odometry (`odom_profile`) | `field_13hz` | Gazebo 20 Hz, no delay | MEASURED, receive times only, 22 bags of 2026-10-27/28: 25 Hz bursts mixed with 4 Hz stretches, 8.6 Hz mean; P(burst→burst) 0.98, P(slow→burst) 0.03 (`timing/odom_timing.py`) | `odom_emulation.py:41-44` |
| `field_4hz` | — | — | MEASURED: same bags without the sonar processor's message-rate request, 3.9 Hz, interval 0.21–0.29 s (5–95 %) | `odom_emulation.py:33` |
| `field_3hz` | — | — | MEASURED on the 2026-09-07 FIT controller logs: 3.0 Hz (0.30 / 0.35 s seen at 20 Hz) | `odom_emulation.py:35` |
| Transport delay | deciles 0 – 161 ms, median 2 ms | none | MEASURED above the fastest delivery (ATTITUDE `time_boot_ms`, 21 604 messages, `timing/link_delay.py`). The constant part cannot be measured without a common clock: `odom_base_delay` 0 s, ASSUMED, bracket 0 – 0.10 s | `odom_emulation.py:37`, `simulation_interface.py:116` |

---

## 7. Controller columns (`gain_profile`, `master_control.py:412`)

`''` / `field` = the real boat's column; `sim` = the sim-tuned legacy column (simulation only). Values: `FIELD_TUNING.md`.

| Group | Real column (field-equivalent) | Sim-tuned (legacy) | Origin of the real column |
|---|---|---|---|
| PID outer x / ψ, inner u / r | [3.5, .01, 0] / [3.3, .01, 0] / [1.0] / [1.5] | [8, .01, 0] / [4, .01, 0] / [2.5] / [2.5] | field tuning, retuned 2026-10-27 [DOC `FIELD_TUNING.md`] |
| LoS `los_ku`, `los_kpsi`, speed scale | 30, 4.5, 1.0 | 30, 4.5, 2.0 | same |
| MPC model | m 16.01, iz 5.64, a (−26.77, −7.55, −21.77), d (−29.34, −51.54, −44.65) | secant of the BlueROV2 plant | in tree since the beginning, provenance unknown [DOC] |
| MPC horizon / time / R / Q | 15 / 2.5 s / 0.015 / (50, 50, 30, 1, 1, 1) | 30 / 6.0 s / 0.10 / same | as flown |
| Thrust limit | 20 N | 20 N | — |
| Point laws (`point_k_v`, `point_k_psi`, `safety_distance`, `manual_hold_kx`, manual surge law) | 0.15, 100, −1, 8, 10·ln | 2.0, 60, 1, 15, 7·ln | field |

Per-day gains for re-flying field runs: `blueboat_control/config/field_runs/` (index in its `README.md`).

---

## 8. Mission-duration effect of the hull interval (R4)

20-minute lawnmower at 1.0 m/s (10 lines of 100 m, 10 m apart, 5 m half-circle turns; route time 1139 s), MPC,
field-equivalent column, offline harness plant (`test/mc_harness.py --plant …`), calm, ideal odometry. `r4/`.

| Plant | Completed in | Against the route time | Thrust per metre |
|---|---|---|---|
| identified | 1201 s | +5.4 % | 22.1 N·s/m |
| slow end (23.7 N for 1 m/s, largest inertias and yaw damping) | 1220 s | +7.1 % | 27.1 N·s/m |
| fast end (18.6 N for 1 m/s, smallest inertias and yaw damping) | 1139 s | 0.0 % | 18.3 N·s/m |

The hull interval moves the duration of that mission by −5.2 % … +1.6 % around the identified plant. On the
straight lines both ends hold 1.0 m/s (20 N of the 40 N available); the difference is made in the turns.
The PID with the real gains does not complete this 1.0 m/s route within 1350 s on any of the three plants (it
falls behind its target by 1.5–2.7 m); the field PID runs were flown on 0.5 m/s routes.
