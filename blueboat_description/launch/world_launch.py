from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():
    
    sl = SimpleLauncher()
    sl.declare_arg('gui', default_value=True)
    sl.declare_arg('spawn', default_value=True)
    sl.declare_arg('thr',default_value = 'thrusters_ur')
    # Spawn heading in RADIANS (Gazebo -Y). Forwarded into upload_rov_launch's
    # declared gazebo axes; x/y stay at their defaults deliberately -- the
    # pre-deployment hold pose is world (0, 0), so a non-origin spawn would
    # make the boat transit there. (Replaces the old dead 'spawn_pose' arg,
    # which upload_rov_launch never read.)
    sl.declare_arg('yaw', default_value=0.)
    # Forwarded to upload_rov_launch.py (2026-11-13): the plant of the description
    # (identified | bluerov2_legacy) and where Gazebo's odometry is bridged.
    sl.declare_arg('plant', default_value='identified')
    sl.declare_arg('odom_raw', default_value=True)
    # Who owns heave / roll / pitch of the identified plant (2026-11-15, R-8; blueboat.xacro): sea_node
    # (default, the simulator's sea node) | standalone (Sim_launch.py: Gazebo's box + plugin damping)
    sl.declare_arg('vertical_model', default_value='sea_node')

    with sl.group(if_arg='gui'):
        sl.gz_launch(sl.find('blueboat_description', 'world.sdf'), "-r")
        
    with sl.group(unless_arg='gui'):
        sl.gz_launch(sl.find('blueboat_description', 'world.sdf'), "-r -s")

    bridges = [GazeboBridge.clock(),
               GazeboBridge('/ocean_current', '/current', 'geometry_msgs/Vector3',
                            GazeboBridge.ros2gz)]
        
    sl.create_gz_bridge(bridges)

    with sl.group(if_arg='spawn'):
        sl.include('blueboat_description', 'upload_rov_launch.py',
                   launch_arguments={'thr': sl.arg('thr'), 'yaw': sl.arg('yaw'),
                                     'plant': sl.arg('plant'), 'odom_raw': sl.arg('odom_raw'),
                                     'vertical_model': sl.arg('vertical_model')})
    return sl.launch_description()
