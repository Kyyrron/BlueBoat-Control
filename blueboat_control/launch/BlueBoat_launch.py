from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)

sl_motors = sl.declare_arg('enable_motors', default_value = False)              # Safety measure to test the program without sending inputs to the robot
sl_note = sl.declare_arg('note', default_value= '')                             # Add additionnal info on the log name
sl_controller = sl.declare_arg('controller_type', default_value = '')           # As an option, it is possible to launch a controller along the robot interaction
sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping') # Trajectory reference for the control
sl_pinger = sl.declare_arg('use_pinger', default_value = False)                 # Tell the controller to rely on pinger coordinates or computed trajectory

def launch_setup():

    # Connect to the robot
    sl.node('mavros',        # package
            'mavros_node',   # executable
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@192.168.2.2:14550',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1
            }])

    # Robot interaction
    sl.node('blueboat_control', 
            'robot_interface.py', 
            parameters={'enable_motors' : sl_motors,
                        'note' : sl_note,
                        'controller_type' : sl_controller,
                        'use_UWgps': sl_pinger,
                        'use_sim_time': False})

    # Connect to the pinger and log data
    sl.node('blueboat_control', 
            'uwgps_log.py', 
            parameters={'use_sim_time': False})

    # Handle parameter setting (mainly used to override motor control for teleoperation)                                       
    sl.node('blueboat_control', 
            'param_set.py')

    # Check if controller is enabled
    if sl.arg('controller_type') != '':

        # Controller
        sl.node('blueboat_control', 
                'master_control.py', 
                parameters={'controller_type' : sl_controller,
                            'simulation' : False,
                            'use_pinger': sl_pinger,
                            'use_sim_time': False})

        if sl.arg('use_pinger') == False:
                # Compute trajectory and target
                sl.node('blueboat_control', 
                        'path_generation.py', 
                         parameters={'trajectory' : sl_trajectory})


    return sl.launch_description()

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
