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
