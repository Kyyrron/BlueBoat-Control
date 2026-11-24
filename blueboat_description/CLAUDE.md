# CLAUDE.md — `blueboat_description`

Loaded when working under `blueboat_description/`. Moved verbatim from
`BlueBoat-Control/.claude/CLAUDE.md` §8 on 2026-10-08 (section references point there).

The URDF/xacro model, meshes, Gazebo world, and the spawn/bridge launch chain
`world_launch.py` → `upload_rov_launch.py` → `state_publisher_launch.py`.
`world_launch.py` declares `yaw` (radians, default 0) and forwards it into
`upload_rov_launch.py`'s `declare_gazebo_axes` arguments, reaching the spawn as
`ros_gz_sim create … -Y <yaw>`; its old `spawn_pose` argument was dead (never read
downstream) and is removed.

**Two plants (2026-11-13), xacro argument `plant`**, threaded `world_launch.py` →
`upload_rov_launch.py` → `state_publisher_launch.py` like `replay`:

- `identified` (**default**) — `urdf/hydrodynamics_identified.xacro`: surge and yaw identified
  on field logs, sway derived from the vendor CAD hull, heave / roll / pitch added inertias from
  R-8 (2026-11-15, the simulator's derivation) with the vertical damping and the CoG selected by
  the xacro argument `vertical_model` (below). It also states the
  **actuator model** (`thrust_gain_right/left`, `prop_speed_loss`) inside the plugin element:
  Gazebo ignores those tags, `blueboat_control`'s `simulation_interface` reads them from
  `robot_description` and applies them to every command. The link's yaw inertia is the second
  `<inertial>` block of `blueboat.xacro`.
- `bluerov2_legacy` — `urdf/hydrodynamics.xacro` and the first `<inertial>` block: the
  description of before, identical apart from xacro's banner line (checked on its three
  variants). A `replay:=true` description carries no plant and is the same for both.

Numbers, origins and intervals: `docs/PARAMETER_REGISTER.md`. Three things to know before
touching either file:

1. **The inertial frame is rolled by π/2** (`rpy="${pi/2} 0 0"`), so the link's **yaw** inertia
   is the attribute `iyy`, and `izz` (5.64) is **pitch**. The simulator's smoke test reads the
   **first** `<inertial>` block's literals: keep that block first and literal.
2. **Gazebo rejects a link whose inertia breaks `iyy ≤ ixx + izz`** ("invalid inertia", the boat
   then simply does not spawn). The identified total yaw inertia (9.48 kg·m²) is therefore
   split: 7.50 rigid + 1.98 as `nDotR`.
3. **gz-sim 8.15's hydrodynamics plugin applies the added-mass Coriolis terms with the opposite
   sign to Fossen's** (`buildCoriolisMatrix`, applied as `+Cmat·ν`): the sway–yaw coupling
   restores instead of destabilising. `nR` / `nRabsU` were fitted on the plant as integrated;
   changing `yDotV`, `yVabsV`, the plugin version or `<disable_coriolis>` needs a refit. The
   plugin accepts cross terms (`nRabsU` = yaw damping per m/s of surge).
4. **The plugin's added mass is explicit, and diverges once it is large** (2026-11-15). It is
   applied from the previous step's acceleration, so the description integrates only while the
   spectral radius of M_rb⁻¹·M_A **at the link origin** is < 1. The CoG offset couples sway and
   roll there. R-8's set gave 1.15–1.30, and the boat diverged in Gazebo within 15 s. Hence
   `zDotW` is capped at −14.5, A44 / A55 are rigid inertia in `ixx` / `izz` (`kDotP`, `mDotQ`
   0), and `yDotV` stays −14 (the CAD derivation gives 133 kg). Check ρ before raising any
   added mass (`docs/PARAMETER_REGISTER.md` §3).
5. **`vertical_model`** (`sea_node` default | `standalone`; threaded like `plant`). `sea_node`:
   the simulator's sea node owns hydrostatics and all of the vertical damping (plugin `zW kP
   mQ` = 0) and the identified CoG is the physical one (`z_cog_physical`, +0.1376).
   `standalone` (`Sim_launch.py`): Gazebo's buoyancy box is the hull, the plugin carries the
   damping, and the CoG stays at the box construction `z_cog`; the physical CoG capsizes the
   box (roll GM −0.025 m). The legacy plant ignores the argument.

`upload_rov_launch.py odom_raw` (default True) bridges Gazebo's odometry to `<ns>/odom_gz`
instead of `<ns>/odom`; `simulation_interface` republishes `/blueboat/odom` from it. False
restores the direct bridge.

Hull (legacy block): `mass = 16.01` kg (`blueboat.xacro:36`), `iyy = 3.9235025` — the yaw
value Gazebo integrates — and `izz = 5.6403125` (`blueboat.xacro:61`), which is pitch and which
the lines below (written before the frame was noticed) call the yaw inertia.
`master_control` hands the MPC `robot_mass = 16.01` and `iz = 5.64` — the mass agrees exactly,
the yaw inertia is the URDF value rounded (0.006 % low). `hydrodynamics.xacro` carries the
BlueROV2 table (`xDotU -5.5`, `xU -25.15`, plus quadratic damping). `master_control`'s
**simulation** `mpc_model` is fitted to it (added mass = the xacro values, damping = secant
linearisations of its quadratic drag, `_declare_tuning_parameters`); the **real-boat** column
(`a_u = -26.77` … `d_r = -44.65`, provenance unknown) shares only the rigid body. Even in
simulation the MPC's linear-damping model is an approximation of the plant, not the plant. Thrusters
sit at `x = -0.488`, `y = ∓0.295`, `z = -0.025`, with `thruster1` on the starboard side (§5).
`thrusters_ur` (2 thrusters) is the default; `thrusters_uvr` (3) exists and is marked not
functional.

The package holds nothing else. The vendored Plankton sensor snippets (`urdf/snippets/`),
`urdf/path_markers.sdf` and the `slider_publisher` configs (`launch/manual.yaml`,
`launch/thrusters.yaml`) were deleted in the 2026-09-14 stabilization pass: nothing in the
superproject included any of them, and `blueboat.xacro`'s only sensor include had been
commented out. ⚠ One live oddity survives that deletion: `upload_rov_launch.py` defaults
`sliders:=True` and looks for `<thr>.yaml` = `thrusters_ur.yaml`, which exists nowhere — and
`Sim_launch.py` passes `sliders: False` to `world_launch.py`, which neither declares nor
forwards it. So `slider_publisher` is requested on every simulation launch and can never find
its config. Fixing it is a launch-behaviour change and has not been made.

**`replay` (v1.1, 2026-10-13).** `blueboat.xacro` declares `replay` (default `false`;
the default output is unchanged), forwarded `upload_rov_launch.py` → `state_publisher_launch.py`
and set only by `blueboat_control`'s `Replay_launch.py`. When `true` **no force can move the
hull**: no Hydrodynamics include, no Thruster plugins (`thrusters_ur.xacro`), no collision
geometry (the world's Buoyancy acts on collision volumes), `<gravity>0` on every link, and
`gz-sim-velocity-control-system` with no command, which pins the model's velocity to zero
every step. That last one is load-bearing: `set_pose` does not reset velocity, so the
simulator's sea-state hull wrench (ApplyLinkWrench, injected at run time) still accumulated
0.16 m of drift in 2 s without it (0.8 mm with it). A **static** model is not the fix:
UserCommands' `set_pose` does not hold a static model. `upload_rov_launch.py` with `replay`
bridges no odometry (`bag_replay` publishes `/blueboat/odom`) and no thruster commands, and
keeps `pose_to_tf` on `base_link` for RViz.

`upload_rov_launch.py` is where simulation gets its sensing: it bridges Gazebo's odometry to
`/blueboat/odom` (the `OdometryPublisher` plugin runs at 20 Hz with `odom_frame: world`),
plus `/blueboat/pose_gt`, `joint_states` and `cmd_thruster{1,2}`. So in a real-robot run
`/blueboat/odom` comes from `robot_interface`, and in simulation it comes from the bridge —
same topic, same type, different origin.
