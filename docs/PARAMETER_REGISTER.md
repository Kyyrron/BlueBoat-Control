# Parameter register — BlueBoat-Control

Every result-affecting number of the simulated plant, of its actuator and timing models, and of the
controller columns: value, unit, origin, interval or bracket, where it lives. Written 2026-11-13/14
(session 04 of the simulator completion round, acceptance test I6); heave / roll / pitch, CoG and sway revised
2026-11-15 (session 04b, R-8; evidence `~/ros2_ws/sim_runs/sim_acceptance/04b_control_r8/`, `handoff_04b.md`).

**Origin tags.** MEASURED = read from our own field logs. IDENTIFIED = fitted on FIT field logs (a kind of
MEASURED; the interval is a 95 % bootstrap interval over segments). VENDOR = the manufacturer's published
figure (URL in `REFERENCES.md` §V). DERIVED = computed from other entries by a stated rule. ASSUMED = neither,
always with a bracket. LEGACY = carried over unchanged from the BlueROV2 table or the third-party model file.
FORCED = a value the physics engine cannot integrate is carried in another form or capped (reason, what is lost and
the evidence are in the row).

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
| Yaw: time constant at rest | 1.50 s (1.49 s [1.34, 2.87] with the 2026-11-15 sway drag) | [1.30, 2.99] | `ident/yaw_fit5.json`; `04b_control_r8/fit/yaw_fit5_r8_fixIzz.json` |
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

**Stability bound of the plugin's added mass (2026-11-15) [VERIFIED].** gz-sim 8.15 applies the added-mass force
from the previous step's acceleration, `stateDot = (state − prevState)/dt`
(`04_control/inventory/downloads/gz_sim8_hydrodynamics/Hydrodynamics.cc`; the plugin warns about it at load). One
step multiplies an acceleration error by −M_rb⁻¹M_A, so the description is integrable only while the spectral
radius ρ(M_rb⁻¹M_A) < 1, with M_rb taken at the **link origin** (the CoG offset couples sway and roll). R-8's set
in the plugin gave ρ = 1.15–1.30 over the CoG bracket. In Gazebo the boat diverged in 12–15 s with zDotW −16.15,
and in roll at 12 s with zDotW −14.5 (`04b_control_r8/gz/r8_zDotW_16p15_external`, `r8_zDotW_14p5_external`). The
carried set has ρ = 0.930 (box CoG) and 0.937 (physical CoG). Hence three FORCED rows below.

| Coefficient | Tag | Value | Unit | Origin | Interval / bracket | Where |
|---|---|---|---|---|---|---|
| Rigid mass `m` | `mass` | 16.01 | kg | LEGACY (the simulator reads it from the robot description) | vendor: 14.5 kg bare + 2.4 kg per two battery packs + 0.3 kg per sonar head = 17.5 kg (2 packs) to 19.8 kg (4 packs), before mounts and cables; survey loading never weighed → 16 – 20 kg | `blueboat.xacro:36` |
| Surge added mass `A11` | `xDotU` | −1.60 | kg | ASSUMED: 10 % of the rigid mass (slender hull) | 5 – 15 %; with κ it reproduces the identified 35.7 kg per commanded N | `hydrodynamics_identified.xacro:48` |
| Surge drag, linear | `xU` | 0.0 | N·s/m | IDENTIFIED (fits to zero) | — | `:56` |
| Surge drag, quadratic `Xuu` | `xUabsU` | −4.23 | N·s²/m² | IDENTIFIED: κ × 8.58 | κ × [2.6, 19.7]; tied to `kp` (§1.1) | `:64` |
| Sway added mass `A22` | `yDotV` | −14.0 | kg | FORCED (2026-11-15). The same rule at the CAD draft, ρπT(x)²/2 per demihull over the CAD stations at 0.212 m, gives **132.6 kg** (13.6 kg was the 0.06 m draft). That is 8 × the rigid mass and not integrable by the plugin (bound above), so 14.0 is kept. Lost: sway inertia and the (A22−A11) u v coupling above 14 kg. The FIT data do not constrain sway: yaw-rate rms 0.0789–0.0805 rad/s over 8–30 kg (`ident/yaw_fit5.json`, `04b/fit/yaw_fit5_r8.json`) | 8 – 133 kg (Gazebo: ≤ 14) | `hydrodynamics_identified.xacro` `yDotV` |
| Sway drag, linear | `yV` | 0.0 | N·s/m | ASSUMED (no lift term) | — | `:57` |
| Sway drag, quadratic `Yvv` | `yVabsV` | −319.1 | N·s²/m² | DERIVED (2026-11-15): cross-flow ½ρC_dA, C_d 1.5 on the CAD lateral area 2 × 0.2127 m² at 0.212 m draft (`03_sea_state/r8/cad_hydrostatics.json` stations). Was 108.0 on 0.144 m² (draft 0.06 m, outside the CAD draft) | C_d 1.0 – 2.0: 213 – 426 | `yVabsV` |
| Yaw inertia, total `Izz` | `iyy` + `nDotR` | 7.50 + 1.98 = 9.48 | kg·m² | IDENTIFIED (total). The split was forced by `iyy ≤ ixx + izz` (8.06 in session 04; 10.78 since the roll / pitch rows below, the split is kept unchanged). Held at 9.48 in the 2026-11-15 refit (a free refit gives 9.50 [8.22, 10.71]) | [8.11, 10.95] | `blueboat.xacro` identified `<inertial>` `iyy`, `nDotR` |
| Yaw damping at rest `Nr` | `nR` | −6.35 | N·m·s/rad | IDENTIFIED; refit 2026-11-15 for the new `yVabsV`, Izz held (`04b/fit/yaw_fit5_r8_fixIzz.json`; was −6.31) | [3.30, 7.08] | `nR` |
| Yaw damping, quadratic | `nRabsR` | 0.0 | N·m·s² | IDENTIFIED (fits to zero) | — | `:69` |
| Yaw damping per m/s of surge `Nur` | `nRabsU` | −5.12 | N·m·s/rad per m/s | IDENTIFIED together with the sway coupling above; refit 2026-11-15 (was −4.76). Yaw prediction on the six FIT runs unchanged: correlation 0.907–0.942 (was 0.907–0.940; `04b/fit/yaw_crosscheck_r8.json`) | [1.33, 15.5]; 4.73 – 5.53 across the Yvv bracket, 7.40 at A22 8 kg | `nRabsU` |
| Heave added mass `A33` | `zDotW` | −14.5 | kg | FORCED (2026-11-15). R-8 derives 16.15 kg (1.0 × displacement, MSS `otter_legacy.m` rule; `03_sea_state/R8_request_for_control.md` §2.3). At 16.15 = the moving mass the plugin diverges (bound above), so the value is capped at 0.9 × 16.15. Heave f_n 1.35 Hz on the CAD hull (K33 2199 N/m), 1.02 Hz on Gazebo's box (1264 N/m) | R-8: 8.1 – 24.2 kg (Gazebo: < 16.15) | `zDotW_identified` |
| Roll inertia, rigid + added | `ixx` (link roll) + `kDotP` | 4.21369 + 0 | kg·m² | FORCED (2026-11-15): rigid 2.42369 (LEGACY, third-party model file) + R-8's A44 1.79 (DERIVED: A33 · y², y = 0.333 m CAD waterplane centroid), carried as **rigid** inertia because the plugin term is unstable with it (bound above; roll diverged at 12 s). I + A is R-8's. Lost: the Coriolis terms of A44 | A44 0.90 – 2.69 | `blueboat.xacro` identified `<inertial>` `ixx`; `kDotP` 0 |
| Pitch inertia, rigid + added | `izz` (link pitch) + `mDotQ` | 6.5703125 + 0 | kg·m² | FORCED (2026-11-15): rigid 5.6403125 (LEGACY, third-party) + R-8's A55 0.93 (DERIVED: strip a33(x) ∝ b(x)², CAD stations), as for roll. The inertial frame is rolled by π/2, so attribute `izz` is the link's pitch axis and `iyy` its yaw: link diag = R_x(π/2) diag(ixx, iyy, izz) R_xᵀ = (ixx, izz, iyy) (`04b/gates/inertia_axis_proof.txt`). Lost: the Coriolis terms of A55. Rigid pitch KEPT (finding c): no source for another value; the 2.3 – 3.3 kg·m² inferred from corpus spectra came from HELD-OUT files and cannot change a value | A55 0.47 – 1.40; rigid pitch: no sourced bracket | `izz`; `mDotQ` 0 |
| Centre of gravity height | identified `<inertial>` origin z | `vertical_model:=sea_node` **+0.1376**; `standalone` −0.12915 | m (link z) | sea_node: DERIVED (2026-11-15) from a mass budget (`04b/tools/cog_budget.py`, `04b/fit/cog_budget.json`): vendor masses (14.5 kg bare, motors, ESCs, batteries, Omniscan heads), CAD hull shell (3.733 L, area centroid 0.218 m above the keel), CAD motor height 0.105 m; ASSUMED: PE density 0.91–0.97, batteries 0.17–0.26 m above the keel, the remaining 6.8 kg (crossbars, tube, lids, electronics) at 0.22–0.38 m. 0.239 m above the keel; keel at link z −0.101 (link origin 0.111 m below the waterline at Gazebo's trim, CAD draft 0.212 m). The old −0.129 is the buoyancy-box construction, 0.028 m below the CAD keel. standalone: kept at that construction, because Gazebo's box is then the only hull and the physical CoG puts its roll GM at −0.025 m (capsizes) | +0.097 … +0.178 (0.198 – 0.279 m above the keel) | `blueboat.xacro` `z_cog_physical`, `z_cog_identified` |
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

## 4. Heave, roll and pitch: who owns what (R-8, 2026-11-15)

The nine vertical plugin entries left the LEGACY table on 2026-11-15. Added inertias are §3's (DERIVED by the
simulator's session 03, FORCED where the plugin cannot integrate them). Damping and hydrostatics depend on the
xacro argument **`vertical_model`** (`blueboat.xacro`, threaded `world_launch.py` → `upload_rov_launch.py` →
`state_publisher_launch.py`):

| `vertical_model` | Used by | Hydrostatics | Damping in the plugin (`zW`, `kP`, `mQ`; quadratic all 0) | CoG |
|---|---|---|---|---|
| `sea_node` (default) | every world run (`full_mission_launch.py` includes `upload_rov_launch.py`) | the simulator's CAD catamaran (`sea/hull.py`, H_cat − H_gz) | **0, 0, 0**: the sea node carries all of it, b = 2ζ√(K(I+A)) on its own hull (`sea/motion.py:damping_from_ratios`) | physical, +0.1376 |
| `standalone` | `blueboat_control` `Sim_launch.py` (its default; it starts no sea node) | Gazebo's buoyancy box | **−118.09, −5.086, −6.952** (same rule, ζ 0.30 / 0.20 / 0.20 of R-8 §2.4, on the box: K 1264 N/m, 38.4, 46.0 N·m/rad; I + A 30.65 kg, 4.214, 6.570) | box construction, −0.12915 |

Exactly once, every path [VERIFIED in code]: the sea node reads the plugin's linear damping from
`/blueboat/robot_description` (`ros/sea_state_node.py` → `sea/plant.py:parse_urdf`) and adds
`max(0, b_target − b_plugin)` (`sea/motion.py:damping_for_zeta`). With `sea_node` it carries the whole damping,
and with `standalone` + an MCS companion only the complement. Without a sea node, the plugin's damping is all
there is.

| Still LEGACY | Value | Why kept |
|---|---|---|
| `ixx`, `izz` rigid parts, mass, buoyancy box | 2.42369, 5.6403125, 16.01 kg, 1.2 × 0.91 × 0.376 m | no source for other values. The simulator's smoke test reads the first (legacy) `<inertial>` block literally |

Ownership of the degrees of freedom: surge, sway, yaw: this module (§3). Heave, roll, pitch: this module's
inertias, plus the simulator's hull wrench (hydrostatics and, under `sea_node`, all of the damping). The simulator
adds no surge, sway or yaw damping (`sea/hull.py`, `docs/sea_state_model.md`), so nothing is counted twice.

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

Computed 2026-11-14 with the sway of session 04 (`Yvv` 108, `Nr` 6.31, `Nur` 4.76). Not re-run on the
2026-11-15 sway (the harness's `identified` variant now mirrors it; `identified_slow` / `_fast` are still session
04's).

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

## 9. Pinger filter (`pinger_estimator`, 2026-11-24)

Every parameter (unit, default, range, what it does) is in `blueboat_control/config/pinger_filter.yaml`;
this table gives where the main values come from. "Provisional" = to be re-fitted by the live field
calibration (`FIELD_TUNING.md`, "Pinger filter"). Fixed in `pinger_filter.py`, not parameters: 30 s pose
history, 0.5 s pose extrapolation, 2 s maximum odometry gap (odometry ran at 3.8–16 Hz in the field), 1 s
estimate heartbeat, 60 s statistics window.

| Parameter | `slow_rov` value | Origin | Interval / note |
|---|---|---|---|
| `latency_s` | 1.5 s | pooled local consistency of raw fixes on 20 pool bags (flat 1.0–1.75 s); literature 1.5 s | **provisional**; WL filtered lags ≥ 1 s more |
| `y_sign` | +1 | world consistency (flip worse on 19/20 runs), pool outline (14/20) | **provisional** until a still-pinger run |
| `mount_yaw_deg` | 0° | per-run regression −9…+9° | not identifiable on a moving target; **provisional** |
| `lever_x_m`, `lever_y_m` | 0 | unknown (4 receivers at the boat's corners, reference point unknown) | **provisional** |
| `sigma_bearing_deg` | 12° | tuning sweep (flat 10–14°); field residual 13–17° median | spec 1° |
| `range_k` | 0.2 | tuning sweep; along-range residual ≈ 20 % of range | spec 0.002 |
| `sigma_a` | 0.02 m/s² | tuning sweep (0.02–0.03) | crawler 0.1–0.3 m/s |
| `v_max` | 0.5 m/s | owner: the brush robot crawls at 0.1–0.3 m/s | `static_target`: 0 |
| `gate_soft`, `gate_hard` | 5.99, 18.42 | χ²₂ 95 %, 99.99 % | |
| `v_hard` | 1.5 m/s | no effect in the sweep (1.0–3.0) | |
| `init_n`, `init_radius_m` | 3, 3 m | design | `static_target`: radius 5 m |
| `reinit_n`, `reinit_radius_m`, `reinit_window_s` | 4, 2 m, 6 s | A1: a 15 m relocation is re-acquired in 4 fixes | |
| `coast_after_s`, `stale_timeout_s` | 2 s, 6 s | device ~1.6 fixes/s; dropouts | |
| `pinger_stale_timeout` (`master_control`) | 3 s | 3 × the 1 s estimate heartbeat | follow mode holds beyond it |
| `raw_rate_hz`, `full_rate_hz` (`uwgps_log`) | 4 Hz, 2 Hz | the legacy packet stays on its 2 Hz cycle | device ≥ 1.7 Hz |
| `connect_timeout_s`, `read_timeout_s` (`uwgps_log`) | 0.25 s, 0.4 s | one cycle median 30 ms, p95 0.1–1.0 s; one 103 s hang without a timeout | |
