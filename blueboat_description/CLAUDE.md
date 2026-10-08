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
(`master_control.py:359-366`). So the Gazebo plant and the MPC's internal model share the rigid
body and nothing else — an MPC result in simulation is not a solver-against-its-own-model
result the way the offline harness's is. Thrusters
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

`upload_rov_launch.py` is where simulation gets its sensing: it bridges Gazebo's odometry to
`/blueboat/odom` (the `OdometryPublisher` plugin runs at 20 Hz with `odom_frame: world`),
plus `/blueboat/pose_gt`, `joint_states` and `cmd_thruster{1,2}`. So in a real-robot run
`/blueboat/odom` comes from `robot_interface`, and in simulation it comes from the bridge —
same topic, same type, different origin.
