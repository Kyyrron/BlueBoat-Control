# References — live path update

Sources behind the design choices of the live path update
([PATH_UPDATE_DESIGN.md](PATH_UPDATE_DESIGN.md)). Where a source is cited, its pointer (Rn plus a
section or equation) is the same here, in the design doc and in the code comments.

**Status legend:**
- `[full]` — read in full.
- `[partial]` — only the stated parts read.
- `[abstract]`.
- `[not accessed]`.

**Rule.** A source not read in full is used only for the parts that *were* read. Nothing was
reconstructed from memory.

## External sources

### R1 — Faulwasser & Findeisen (2016)

T. Faulwasser, R. Findeisen. *Nonlinear Model Predictive Control for Constrained Output Path
Following.* IEEE Transactions on Automatic Control 61(4):1026–1039, 2016.
doi:[10.1109/tac.2015.2466911](https://doi.org/10.1109/tac.2015.2466911).

**Status: `[partial]`.**
- Read as preprint arXiv:1502.02468v1: §I–VI in full.
- Appendix A cut off at §C by the download.
- The published version was not read.

**Used:**
- **§II, Problem 1(ii).** Forward motion θ̇ ≥ 0. Applied as τ monotonic (design D-2).
- **Remark 1 and eq. (6d).** The path parameter is an *internal state of the controller*,
  carried from one sample to the next. Applied as: expose τ as the progress and create no
  second notion of progress (D-2).
- **Eq. (6e).** The OCP is posed on p(θ̄(τ)) over the whole horizon [t_k, t_k+T]. Applied as
  the floor term `path_time` of the minimum lead, i.e. the MPC horizon (D-4).
- **Eq. (8b).** θ(t₀) = argmin‖h(x₀) − p(θ)‖ when no initial condition is given. Applied as:
  a first path must start at the hold pose (D-5).

### R2 — Lapierre & Soetanto (2007)

L. Lapierre, D. Soetanto. *Nonlinear path-following control of an AUV.* Ocean Engineering
34(11–12):1734–1744, 2007.
doi:[10.1016/j.oceaneng.2006.10.019](https://doi.org/10.1016/j.oceaneng.2006.10.019).

**Status: `[full]`** of the author's accepted manuscript (ENSTA copy). The final typeset version
was not read.

**Used:**
- **§II.B.** A virtual target whose rate ṡ is a control variable, not the projection of the
  vehicle onto the path. Applied as: τ is that target's parameter (D-2).
- **Propositions 1–2.** They need ψ_F, c_c and ∂c_c/∂s to be well defined, because the law
  feeds curvature forward. Applied as: none of our three laws uses curvature feed-forward, so
  C¹ (position and tangent) continuity at a junction suffices (D-4).
- **§V.A.** Followability under saturation is a path-design property. Applied as: feasibility
  is the Replanner's duty; Control applies only sanity bounds (D-4, interface §6).

### R3 — Macenski et al. (2020)

S. Macenski, F. Martín, R. White, J. Ginés Clavero. *The Marathon 2: A Navigation System.*
IEEE/RSJ IROS 2020. arXiv:[2003.00368](https://arxiv.org/abs/2003.00368)v2.

**Status: `[full]`.**

**Used:**
- **§III.B.** The task-specific server "processes cancellation, preemption, or new information
  requests". Applied as: one entry point, in the node that owns the task (D-1).
- **§IV.A.** The global planner is re-ticked at 1 Hz while the controller follows. Applied as:
  replacing a followed path in motion is established practice (D-1).
- **§IV.G.** Gazebo system tests in CI. Applied in the Phase 3 test design.

It gives **no** acceptance, lead or continuity criteria, and none are taken from it.

### R4 — Fossen (2021), ch. 12

T. I. Fossen. *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley,
2021. Ch. 12 "Guidance Systems",
doi:[10.1002/9781119575016.ch12](https://doi.org/10.1002/9781119575016.ch12).

**Status: `[partial]`.** The document provided is **the author's lecture notes for chapter 12
(76 slides), not the book chapter**. All slide text was read, and the equation slides 35–37,
44–49 and 55 were viewed.

**Used:**
- **Slide 46, §12.4.2.**
  - Proportional LOS: χ_d = π_h − atan(K_p·y_e), with K_p = 1/Δ.
  - Cross-track dynamics: ẏ_e = −U·y_e/√(Δ²+y_e²).
  - Applied as the step sizes behind both continuity tolerances (D-4).
- **Slide 48 (`LOSchi.m`).** "To handle steps in the course angle command … due to waypoint
  switching" an observer is needed. Our LoS/PID laws have none, so the junction step is
  bounded instead (D-4).
- **Slides 37 and 48.** Along-track waypoint switching, d − |x_e| ≤ R_switch. Applied as:
  activation when τ crosses the change point (D-3).
- **Slide 35.** Feasibility: maximum speed and turning rate. Applied as the speed sanity
  bound (D-4).
- **Slide 38.** A Dubins path carries "a jump in the desired yaw rate". Applied as: curvature
  steps are allowed (D-4).

### R5 — Breivik & Fossen (2004)

M. Breivik, T. I. Fossen. *Path following of straight lines and circles for marine surface
vessels.* IFAC Conference on Control Applications in Marine Systems (CAMS), Ancona, 2004;
IFAC Proceedings Volumes.
doi:[10.1016/s1474-6670(17)31709-3](https://doi.org/10.1016/s1474-6670(17)31709-3).

**Status: `[full]`.**

**Used:**
- **§2.** The geometric task and the dynamic task (D-2).
- **Eq. (7)–(8).** χ_r = atan(−e/Δ) and ė = −U·e/√(e²+Δ²); the lookahead distance Δ (D-4).
- **Proposition 5.** Assumes ψ_d, ψ̇_d, ψ̈_d ∈ L∞, i.e. a smooth heading reference. Applied as
  heading continuity at a junction (D-4).
- **§1.** A time-parametrised tracking point "will then leave the vessel behind". This is why
  τ is governed rather than clocked (D-2).

### R6 — Skjetne, Fossen & Kokotović (2004)

R. Skjetne, T. I. Fossen, P. V. Kokotović. *Robust output maneuvering for a class of nonlinear
systems.* Automatica 40(3):373–383, 2004.
doi:[10.1016/j.automatica.2003.10.010](https://doi.org/10.1016/j.automatica.2003.10.010).

**Status: `[full]`.**

**Used:**
- **§2, eq. (1)–(4).** The geometric task plus a speed assignment θ̇ → υ_s. Applied as the
  progress semantics (D-2).
- **Eq. (39), the gradient update law** θ̇ = υ_s − μV_θ. The path speed is reduced by the
  tracking error, which is the role of `advance_governor` (D-2).
- **Assumption 2.1.** The path and its partial derivatives are bounded. A position or tangent
  jump violates it; a curvature step does not. Applied as the C¹ junction (D-4).
- **§3.2 / Fig. 4.** *"The path definition must be specified a priori, while the speed
  assignment along the path can be modified online."* Applied as: an update may change the
  path only where the controller has not used it yet — the lead rule (D-4).
- **Example 2, eq. (16)–(17).** Non-smooth corners are traced by slowing to near zero. Applied
  as: a heading change is allowed where the current reference is stationary (D-4).

### Considered, not used

Baxevani, Otto, Trembanis, Tanner. *Optimal ASV Path-Following for Improved Marine Survey
Data Quality*, OCEANS 2023 (in the provided folder).

**Status: `[abstract]`.** It is a controller-design paper and says nothing on replacing a path
in motion.

## Plant identification (2026-11-13)

Sources behind `PARAMETER_REGISTER.md`. Every URL was opened on 2026-11-13; the reading notes, tables and
what could not be opened are in `~/ros2_ws/sim_runs/sim_acceptance/04_control/inventory/web_sources.md`
(outside the repository). Marks: **[full]** read in full, **[part]** the named parts, **[abs]** abstract or
landing page only, **[unread]** found, not opened. Regime of interest: a 16–20 kg twin-hull surface vehicle,
1.2 m long, at 0.5–1.0 m/s.

### V — Vendor data

| # | Source | URL | Mark | Used for |
|---|---|---|---|---|
| V1 | Blue Robotics, BlueBoat product page, "Technical Details" | https://bluerobotics.com/store/boat/blueboat/blueboat/ | [part] | 14.5 kg bare; 15 kg of batteries + payload; 2.4 kg per two battery packs; 2 × M200 motor with weedless propeller on Basic ESC 500; 3 m/s; 8.2 kgf static; ≈ 30 W at 1 m/s |
| V2 | BlueBoat public drawing BR-101447 Rev A | https://bluerobotics.com/wp-content/uploads/2023/03/BR-101447_RevA_PUBLIC_DRAWING.pdf | [full] | 1195 × 910 mm, hull height 376 mm |
| V3 | BlueBoat CAD (hull set, full vessel) | https://cad.bluerobotics.com/BLUEBOAT_HULL_SET_BR-101652.zip | geometry measured on the mesh, ±10 mm (not a vendor statement) | demihull 177 × 376 × 1195 mm, keels 583 mm apart, motor axes ≈ 589 mm apart; no mass properties in the files |
| V4 | M200 + weedless propeller chart data | https://docs.bluerobotics.com/thrusters/m200weedless_force_plot.js | [full] | **the thrust table of `custom_functions.generate_interpolator`**: 32 of 33 values identical, 16 V, bollard |
| V5 | Weedless propeller product page | https://bluerobotics.com/store/thrusters/t100-t200-thrusters/weedless-propeller-set/ | [part] | 112 mm, 2 blades, pitch 14° at 75 % diameter; 5.63 / 2.81 kgf forward / reverse at 16 V |
| V6 | T200 public performance data, 10–20 V (2019) | https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx | [full] | voltage sensitivity of the same motor (proxy): +18–35 % thrust at fixed PWM from 14 V to 16.8 V; does **not** match the stack's table at any voltage |
| V7 | Basic ESC (30 A) and Basic ESC 500 pages | https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/ , https://bluerobotics.com/store/thrusters/speed-controllers/besc500/ | [part] | deadband ±25 µs stated for the 30 A ESC only |
| V8 | Blue Robotics Li-ion 4S batteries; Cerulean Omniscan 450 specifications | https://bluerobotics.com/store/comm-control-power/powersupplies-batteries/battery-li-4s-18ah-r3/ , https://docs.ceruleansonar.com/c/omniscan-450/specifications.md | [part] / [full] | 1152 g per pack; 300 g per transducer in air |
| V9 | gz-sim 8.15.0 hydrodynamics system, source | https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/systems/hydrodynamics/HydrodynamicsUtils.hh | [part] `buildCoriolisMatrix`, `buildDampingMatrix` | what the simulated plant integrates (sign of the added-mass coupling, cross terms such as `nRabsU`) |

Transfer: V1–V8 are this boat's own parts. V4 is a bollard figure at one voltage: it transfers to the boat
under way only through the identified `kp` and κ.

### L — Literature (compared with, never copied from)

| # | Reference | DOI / URL | Mark | Finding and transfer to our regime |
|---|---|---|---|---|
| L1 | Sonnenburg & Woolsey, "Modeling, Identification, and Control of an Unmanned Surface Vehicle", J. Field Robotics 30(3), 371–398, 2013 | 10.1002/rob.21452 (resolves) | [abs] | "Nomoto's first‐order steering model provides the best compromise between simplicity and fidelity at higher speeds"; a sideslip lag helps at low speed. Vessel: a 4.8 m rigid-hull inflatable, planing, 22 kn (from the first author's dissertation, [part]). **Structure transfers, numbers do not.** Our yaw is first-order with a gain that falls with speed, in line with a speed-scheduled Nomoto model |
| L2 | von Benzon et al., "An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot", J. Mar. Sci. Eng. 10(12), 1898, 2022 | 10.3390/jmse10121898 (resolves) | [part] Table A1, §4.2–4.3 | The legacy plant's source. Table A1 (BlueROV2 **Heavy**): Xu 13.7, Xu\|u\| 141, Xu̇ 6.36 kg … — these are the values *commented out* in `hydrodynamics.xacro`; the active block (xU −25.15, xUabsU −33.8, xDotU −5.5) is not Table A1; where it comes from was not located [UNCERTAIN]. A submerged box-frame ROV: **does not transfer** to a surface catamaran |
| L3 | Fossen, Handbook of Marine Craft Hydrodynamics and Motion Control, 1st ed. 2011 / 2nd ed. 2021 | 10.1002/9781119994138 , 10.1002/9781119575016 (both resolve) | [unread] | Notation of the plant; the bilinear thruster model (thrust falling with advance speed) that `prop_speed_loss` is a form of — chapter not opened, to be requested |
| L4 | Fossen, MSS toolbox, `otter.m` (Maritime Robotics Otter USV) | https://github.com/cybergalactic/MSS/blob/master/CRAFT/USV/models/otter.m | [full] | 55 kg, 2.0 × 1.08 m twin hull; time constants set by the author, not identified: surge 5 s, sway 0.5 s, yaw 1 s. Comparable craft, three times heavier: our yaw time constant at rest (1.5 s [1.3, 3.0]) is of the same order; our surge transient (≈ 1.2 s at 1 m/s: 35.7 kg over a local drag slope of 28.7 N·s/m) is faster |
| L5 | Paine & Benjamin, online estimation of 1-DOF models, eight Clearpath Heron USVs | arXiv 2308.00782 | [abs] | Surge only, mass + linear / quadratic / cross drag; no coefficient table seen. Same structure as our surge model |
| L6 | Liu et al., "MOSAIC-SV", adaptive identification with field trials on a BlueBoat | arXiv 2610.03898 | [abs] | A BlueBoat study; no coefficient set seen in the abstract |
| L7 | Lievore, M., Padova master's thesis (the source named in `hydrodynamics.xacro`) | https://thesis.unipd.it/handle/20.500.12608/93733 | [unread] (archive refuses automated access) | Title from search snippets only; whether it holds BlueBoat hull coefficients is unknown |

**No published, experimentally identified surge / sway / yaw model was found for the BlueBoat, the Heron, or
any catamaran of 10–40 kg.** Nothing in the plant comes from another vehicle; sway is derived from this hull's
geometry and bracketed.

**Papers to request** (not in `/home/kyyrron/Downloads/research_paper`): L1 full text; L3 chapters 6–7
(manoeuvring models) and the thruster-model section; L7 (to open in a browser); the dissertation behind L1 if
low-speed sideslip matters; a US technical report on a Clearpath USV, https://apps.dtic.mil/sti/pdfs/AD1046879.pdf
([unread], HTTP 403).

## Project documents

### P1 — `project_synthesis.md` (superproject)

**Status: `[full]`** for the relevant sections.

**Used:**
- **§5.1.** The replanner and mission-manager role. Its "MAVLink mission swap" does not match
  the implementation: nothing goes to ArduPilot; `master_control` follows the path on the
  laptop. This is recorded in the handover.
- **§8.1.** The replanner's own rules (resolution, budget, abandonment) are *not* Control's.
- **§8.5.** 1.0 m/s nominal survey speed. Applied as `path_update_max_speed` = 2 × nominal.
- **§8.6.** Offline-first testing. Applied as the deterministic harness.

### P2 — `controller_comparison/controller_comparison_report.tex`

**Status: `[full]`** for table `tab:runs` and §"Why two of the three finish late".

**Used** (twelve field runs, single site):
- Mean cross-track error 0.24–0.49 m.
- PID trails its target 0.7–1.2 m, LoS 1.3–1.4 m, and MPC sits slightly ahead.

Applied as the position tolerance and the lead argument (D-4).

### P3 — module documents

- `blueboat_control/src/TRAJECTORY_SYSTEM.md` §4–§7
- `blueboat_control/src/CLAUDE.md` §4 (governor; C6: SQP_RTI warm start; the hold gate)
- `FIELD_TUNING.md` governor and hold rows

**Status: `[full]`.**

**Used:** τ semantics, the reference window, the `hold_speed` gate, the end-of-path hold, and
the MPC warm start (D-1 to D-4).
