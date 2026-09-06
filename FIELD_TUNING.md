# FIELD TUNING — `master_control.py`

Line numbers = `blueboat_control/src/master_control.py` (`PID/PID.py` where named).
Defaults given as **real / sim**, selected by the `simulation` parameter.


Branch priority: manual target `:779` → path following `:785` → pinger `:868` → zero thrust `:890`.
A manual target overrides the path in **every** `controller_type`.

---

## Shared — all modes

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `min_thrust` | 2.0 N | `:659` | `:1241`, `:1375` | ESC breakaway floor on point-LoS and manual-hold surge; `0.0` = unfloored law exactly. |
| `path_stale_timeout` | 1.0 s | `:669` | `:752` | Older window → governor **holds `tau`** instead of running open-loop. |

### Governor (PID, LoS, MPC path following) — `advance_governor` `:1105`

`tau_dot = path_speed_scale · fac_along · fac_cross`

> The virtual target the controller chases is not played on a clock: it walks the path only as
> fast as the boat keeps up. **Along-track** = how far ahead of the boat the target is; the target
> runs at full speed under `gov_Lmin`, slides down linearly, and freezes at `gov_Lmax`.
> **Cross-track** = how far the boat is off to the side of the line; `gov_Emax` would freeze the
> target on that too, but a boat that is sideways-off is not helped by taking its forward target
> away, so this half ships disabled (`gov_Emax = 0` → factor is always 1).

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `path_speed_scale` | 1.0 | `:415` | `:1138` | Global mission speed multiplier; 0.5 = half the authored speed. |
| `gov_Lmin` | 0.5 m | `:416` | `:1128` | Along-track gap below which the target runs at full authored speed. |
| `gov_Lmax` | 3.0 m | `:417` | `:1128` | Along-track gap at which the target **stops**; linear between the two. |
| `gov_Emin` | 0.5 m | `:428` | `:1132` | Same, cross-track. Inert while `gov_Emax = 0`. |

---

## 1. PID path following — `:833`, law in `PID/PID.py:134`

> **The `hold_*` pair is only for a reference that has stopped moving** (`station_keeping`, a
> finished mission, waiting for a YAML). Normally PID steers along the path tangent — but a
> parked reference has no meaningful tangent, so a boat drifting sideways off it shows no
> along-track error and gets no thrust. Under `hold_speed` the tangent handed to the law is
> swung round to point **at** the target instead, which turns the boat's distance from the point
> into along-track error the normal gains can act on. `hold_radius` is the "close enough" ring:
> the swing fades out inside it so the boat sits still rather than nudging. Both are inert on a
> real path (every trajectory runs faster than 0.05 m/s).

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `outer_gains_x` `[kp,ki,kd]` | `[3.0,0.01,0]` / `[6.0,0.01,0]` | `:433`/`:439` | `PID.py:180` | Along-track error → surge speed correction on top of the path feedforward. `ki` winds up: keep tiny. |
| `outer_gains_psi` | `[3.0,0.01,0]` / `[4.0,0.01,0]` | `:434`/`:440` | `PID.py:183` | Heading error → yaw-rate reference. ↑ = snappier turn onto the path, weaving if too high. |
| `inner_gains_u` | `[1.0,0,0]` / `[2.0,0,0]` | `:435`/`:441` | `PID.py:186` | Surge-speed error → force (N per m/s). |
| `inner_gains_r` | `[1.5,0,0]` / `[2.5,0,0]` | `:436`/`:442` | `PID.py:187` | Yaw-rate error → moment. Too high = thruster chatter. |
| `pid_lookahead` (Δ) | 2.5 m | `:437`/`:443` | `PID.py:175` | `psi_d = gamma_p + atan2(e_y, Δ)`. Small = aggressive cut back to the line, large = damped.|
| `hold_speed` | 0.05 m/s | `:464` | `:840` | Below this authored speed, the tangent is rotated toward the bearing to the point so the along-track term can see the range. |
| `hold_radius` | 0.5 m | `:465` | `:853` | On-station radius; the rotation fades out inside it. |


---

## 2. LoS path following — `los_guidance` `:1145`, entry `:863`

Surge = authored speed feedforward (no along-track integrator); yaw = P-D on heading error.

> **Speeds vs forces — the chain is `speed command → (los_ku) → Newtons`.** LoS asks for a
> *speed*: normally the path's own authored speed, `los_ku` then turns the shortfall between that
> and the boat's measured surge into thrust (`X = los_ku · (u_cmd − u)`).
> **"Hold surge" is an extra speed command added when the reference has stopped moving** — same
> problem as PID above: a parked path point gives zero authored speed, so the boat would sit at
> zero thrust while the current takes it. Under `hold_speed` the law steers straight at the point
> and asks for `los_hold_kx` m/s per metre it is outside `hold_radius`, capped at `los_hold_umax`
> m/s (1 m off → 1 m/s asked, never more than 0.8). That is why `los_hold_kx` is not comparable
> to `los_ku`: it sets *how fast to come back*, `los_ku` sets *how hard to push* to reach any
> speed. Never reverse — the boat turns round first.

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `los_lookahead` (Δ) | 2.5 m | `:446` | `:1163` | `psi_d = gamma_p + atan2(−e_y, Δ)`; small = harder cut onto the line. |
| `los_ku` | 20.0 | `:447` | `:1189` | Surge gain, **N per m/s** of speed error: 0.5 m/s short → 10 N. |
| `los_kpsi` | 10.0 | `:448` | `:1190` | Yaw moment per rad of heading error: 0.2 rad → 2 N·m. |
| `los_kd` | 1.0 | `:449` | `:1190` | Yaw-rate damping. **Raise this first** if `los_kpsi` makes the bow hunt. |
| `los_speed_scale` | 1.0 / **2.0** | `:450` | `:1187` | Scales the commanded speed, not the target's motion — use `path_speed_scale` to slow a mission. |
| `hold_speed` | 0.05 m/s | `:464` | `:1169` | Below it the law steers at the reference **point** and adds hold surge. |
| `hold_radius` | 0.5 m | `:465` | `:1175` | Only range outside it produces hold surge. |
| `los_hold_kx` | 1.0 | `:466` | `:1183` | Hold surge in **m/s per metre** of gap (velocity, fed through `los_ku`). |
| `los_hold_umax` | 0.8 | `:467` | `:1183` | Cap on that hold speed (m/s). |

---

## 3. MPC path following — build `:327`, solve `:792`

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `mpc_horizon` N | 15 / **30** | `:593` | `:196`, `:335` | Prediction steps; sets step `dt = time/N` and QP size `nv = 2N`. |
| `mpc_time` | 2.5 s / **6.0 s** | `:594` | `:195`, `:336` | Prediction span. ⚠ Real boat stays at 2.5 s — solve time never timed on the companion computer. |
| `mpc_Q_diag` `[x,y,psi,u,v,r]` | `[50,50,30,1,1,1]` | `:596` | `:338` | Tracking weights: `x/y` for position, `psi` for heading. |
| `mpc_R_diag` | `[0.015,0.015]` / `[0.10,0.10]` | `:614` | `:339` | Effort penalty in N². Sets where full throttle gets cheaper than the error: **0.015 → 0.49 m, 0.10 → 1.26 m, 0.25 → 2.00 m**. Raise if it saturates. |
| `mpc_qp_iter_max` | 0 = derive | `:637` | `:342` | qpOASES working-set budget, `0` → `max(50, 4·nu·N)` = 240 @ N=30. ⚠ Never pin below `nv = 2N` (finding C6). |
| `mpc_model` (**not a ROS param**) | `:571` / `:576` | — | `:335` | Plant: mass, `iz`, added mass `a_*`, damping `d_*` (secant fits at 0.45 m/s / 0.15 rad/s). Edit the file. |

`/rosout` lines to watch: `MPC solve FAILED …` (`:809`, commands **zero thrust**, boat drifts),
`MPC solve took X ms of the 50 ms tick` (`:821`), `generated and compiled` vs `reused` (`:352`).
Not used: `outer_*`, `inner_*`, `los_*`, `hold_*`, `point_*`, `manual_*`, `min_thrust`.

---

## 4. Manual target — `solve_LoS` `:1195`, hold `manual_keep_location` `:1280`

Active whenever `/blueboat/manual_target ≠ [0,0]`, in **any** `controller_type`.

**Pursuit (off station)**

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `point_k_v` | 0.15 / 2.0 | `:471` | `:1203` | Surge in `v = 5·ln(k_v·d + 1)`:|
| `point_k_psi` | 10.0 / 60.0 | `:472` | `:1201` | Yaw rate per rad of bearing; differential is `±0.295·yaw_rate` (0.2 rad → 0.59 N/side real). |
| `min_thrust` | 2.0 N | `:659` | `:1241` | Breakaway floor, faded over `hold_radius`, killed by `cos(bearing)`|

**Keep-location (on station)**

> Arriving at a manual target is not a stop: the boat parks and answers drift. Two rings with
> hysteresis — come inside `manual_hold_radius` and it holds; get pushed past
> `manual_reacquire_radius` and it drives back. **Unlike LoS above, this law writes Newtons
> straight to the thrusters** (no speed loop, no allocator): `manual_hold_kx` is force per metre
> of gap outside the hold ring (0.5 m out → 4 N on the real boat) and `manual_hold_umax` caps
> it. So do not copy values between `los_hold_kx` (m/s per metre) and `manual_hold_kx` (N per
> metre). Yaw is untouched — only the forward push is replaced.

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `manual_hold_radius` | 1.0 m | `:518` | `:1336`, `:1357` | Arrive inside → hold; also the gap origin. **`<= 0` disables the hold** (pursuit-only, as before). |
| `manual_reacquire_radius` | 2.0 m | `:519` | `:1342` | Pushed beyond → resume pursuit. The gap to the above is the anti-toggle hysteresis. |
| `manual_hold_kx` | 8.0 / 15.0 | `:520` | `:1358` | Hold surge in **Newtons per metre** of gap (≠ `los_hold_kx`, which is m/s). |
| `manual_hold_umax` | derived `kx·(reacq − hold)` = 8 / 15 N | `:523` | `:1358` | Cap. Leave derived so retuning a radius cannot break the handover. |
| `manual_brake_time` | 1.0 s | `:528` | `:1353` | Reverse pulse on each **fresh** arrival; a repeated target does not re-arm it. |

**Pinger branch** (`use_pinger:=True`, `LoS`) uses the same pursuit law but no hold:

| Parameter | Default | Decl | Used | Effect |
|---|---|---|---|---|
| `safety_distance` | **−1.0** / 1.0 | `:476` | `:1263` | Arrival range: reverse 1 s then **zero thrust for the rest of the run**. Negative disables it (real-boat default). |

With `PID` + pinger, the §1 gains apply in a zeroed body frame (`:875`).

---

## Symptom → knob

| Symptom | Mode | Try, in order |
|---|---|---|
| Lags the virtual target | PID | `outer_gains_x[0]` ↑, `inner_gains_u[0]` ↑ |
| Lags **and** thrust saturates | any | `path_speed_scale` ↓ |
| Steady offset parallel to the path | PID / LoS | lookahead ↓, then `outer_gains_psi[0]` / `los_kpsi` ↑ |
| Weaving about the line | PID / LoS | lookahead ↑, then `los_kd` ↑ / `inner_gains_r[0]` ↓ |
| Cuts corners | any | `gov_Lmax` ↓, `los_speed_scale` ↓ |
| Target runs away | any | `gov_Lmax` ↓, `path_speed_scale` ↓ |
| Thruster chatter | PID / LoS | `inner_gains_r[0]` ↓ / `los_kd` ↑ |
| MPC pinned on ±20 N | MPC | `mpc_R_diag` ↑ (0.015 → 0.05 → 0.10), check `mpc_model` |
| MPC solve FAILED in `/rosout` | MPC | `mpc_qp_iter_max` = 0, `mpc_horizon` ↓, `mpc_R_diag` ↑ |
| MPC solve-time warnings | MPC | `mpc_horizon` ↓ with `mpc_time` (keep `time/N` ≈ 0.2 s) |
| Commanded forward, does not move | manual / pinger | check `min_thrust` = 2.0 (W6) |
| Hunts around a manual target | manual | `manual_hold_radius` ↑, `manual_hold_kx` ↓ |
| Blown off a manual target | manual | `manual_hold_kx` + `manual_hold_umax` ↑, `manual_reacquire_radius` ↓ |
| Toggles hold ↔ pursuit | manual | widen `manual_reacquire_radius` − `manual_hold_radius` |
| Stops dead at the pinger | pinger | `safety_distance` = −1.0 |
| Cannot hold on `station_keeping` | PID / LoS | `hold_speed` above the authored speed, then `los_hold_kx`/`umax` or `outer_gains_x` |

