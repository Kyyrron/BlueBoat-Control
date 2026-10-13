# CLAUDE.md — `blueboat_description`

Loaded when working under `blueboat_description/`. Moved verbatim from
`BlueBoat-Control/.claude/CLAUDE.md` §8 on 2026-10-08 (section references point there).

The URDF/xacro model, meshes, Gazebo world, and the spawn/bridge launch chain
`world_launch.py` → `upload_rov_launch.py` → `state_publisher_launch.py`.
`world_launch.py` declares `yaw` (radians, default 0) and forwards it into
`upload_rov_launch.py`'s `declare_gazebo_axes` arguments, reaching the spawn as
`ros_gz_sim create … -Y <yaw>`; its old `spawn_pose` argument was dead (never read
downstream) and is removed.

Hull: `mass = 16.01` kg (`blueboat.xacro:18`), `izz = 5.6403125` (`blueboat.xacro:42`).
`master_control` hands the MPC `robot_mass = 16.01` and `iz = 5.64` — the mass agrees exactly,
the yaw inertia is the URDF value rounded (0.006 % low). The **hydrodynamics do not agree at
all**: `hydrodynamics.xacro` carries the BlueROV2 table (`xDotU -5.5`, `xU -25.15`, plus
quadratic damping the MPC has no term for), while `master_control` passes `a_u = -26.77`,
`a_v = -7.55`, `a_r = -21.77`, `d_u = -29.34`, `d_v = -51.54`, `d_r = -44.65`
(`master_control.py:659-662`, the real-boat `mpc_model`). So the Gazebo plant and the MPC's
internal model share the rigid body and nothing else — an MPC result in simulation is not a
solver-against-its-own-model result the way the offline harness's is. Thrusters
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
