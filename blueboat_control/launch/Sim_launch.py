from simple_launch import SimpleLauncher


def generate_launch_description():
        sl = SimpleLauncher(use_sim_time = True)

        sl_robot = sl.declare_arg('robot_file', default_value='thrusters_ur')           # Choose between 2 and 3 thrusters architecture (not functionnal yet)
        sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping') # Trajectory reference for the control
        sl_controller = sl.declare_arg('controller_type', default_value = 'MPC')        # Controller to be used

        # Launch gazebo and related simulation nodes
        sl.include('blueboat_description', 
                   'world_launch.py', 
                   launch_arguments={'sliders': False, 
                                     'thr': sl_robot})

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
                            'simulation' : True})

        return sl.launch_description()
