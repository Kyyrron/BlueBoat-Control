from simple_launch import SimpleLauncher


def generate_launch_description():
        sl = SimpleLauncher(use_sim_time = True)

        sl_robot = sl.declare_arg('robot_file', default_value='thrusters_ur')           # Choose between 2 and 3 thrusters architecture (not functionnal yet)
        sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping') # Trajectory reference for the control
        sl_controller = sl.declare_arg('controller_type', default_value = 'MPC')        # Controller to be used
        sl_data = sl.declare_arg('data_dir', default_value = '')                        # Root for the controller .npy log. Empty resolves to $BLUEBOAT_DATA_DIR, else the sourced workspace root.
        sl_spawn_yaw = sl.declare_arg('spawn_yaw', default_value = 0.)                  # Boat spawn heading, RADIANS ENU (0 = East). Spawn stays at (0,0).

        # Launch gazebo and related simulation nodes
        sl.include('blueboat_description',
                   'world_launch.py',
                   launch_arguments={'sliders': False,
                                     'thr': sl_robot,
                                     'yaw': sl_spawn_yaw})

        # Simulation interaction
        sl.node('blueboat_control', 
                'simulation_interface.py')

        # Compute trajectory and target
        sl.node('blueboat_control', 
                'path_generation.py', 
                parameters={'trajectory' : sl_trajectory})

        # Display trajectory and target in rviz
        sl.node('blueboat_control', 
                'path_publisher.py')

        # Load controller
        sl.node('blueboat_control', 
                'master_control.py', 
                parameters={'controller_type' : sl_controller,
                            'simulation' : True,
                            'data_dir': sl_data})

        return sl.launch_description()
