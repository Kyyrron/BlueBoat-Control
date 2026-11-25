from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)

sl_motors = sl.declare_arg('enable_motors', default_value = False)              # Safety measure to test the program without sending inputs to the robot
sl_note = sl.declare_arg('note', default_value= '')                             # Add additionnal info on the log name
sl_controller = sl.declare_arg('controller_type', default_value = '')           # As an option, it is possible to launch a controller along the robot interaction
sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping') # Trajectory reference for the control
sl.declare_arg('use_pinger', default_value = False)                             # Tell the controller to rely on pinger coordinates or computed trajectory
sl_fcu = sl.declare_arg('fcu_url', default_value = 'udp://:14550@192.168.2.2:14550') # MAVROS <-> autopilot endpoint. Port 14550 is also QGroundControl's default listen port: a running QGC takes it first and mavros fails to bind, which shows up as an intermittent launch failure. Close QGC, or move this launch to another port.
sl_data = sl.declare_arg('data_dir', default_value = '')                       # Root for run artifacts (position CSV, controller .npy). Empty resolves to $BLUEBOAT_DATA_DIR, else the sourced workspace root.
sl_session = sl.declare_arg('session_mode', default_value = False)             # Run session (the Mission Control Station passes true): connect and idle; path/pinger following and logging only between run start/stop on /blueboat/run_command, one CSV + .npy per run. Needs a controller_type. False = the classic one-launch-one-mission behaviour.
sl_path_source = sl.declare_arg('path_source', default_value = 'mission')       # 'mission' = follow `trajectory` (today's behaviour). 'replanner' = live path update: each run holds at the boat until /blueboat/update_path loads its first path. In a run session `configure` may switch it per run (docs/PATH_UPDATE_INTERFACE.md).
sl_bag = sl.declare_arg('record_bag', default_value = True)                     # Run sessions only (v1.1): record one ros2bag per run (every topic except the side-scan sonar) into the run's Robot_data/<stem>/ folder. A separate `ros2 bag record` process, pre-armed paused between runs.
# Pinger (2026-11-24). pinger_mode: off | follow | log ('' = from use_pinger: true -> follow, false -> off). log = "log the pinger only": the boat never steers to it; with a path it follows the path (any controller), with has_path:=false it only holds a manual target / arrow keys and requests no path. In a session each run's `configure` decides.
sl_pinger_mode = sl.declare_arg('pinger_mode', default_value = '')
sl_has_path = sl.declare_arg('has_path', default_value = True)                 # a run without a path (pinger mode off or log): false (no path server outside a session, no /path_request)
sl_pinger_source = sl.declare_arg('pinger_source', default_value = 'filter')   # what /blueboat/pinger_coordinates carries: filter (default) | legacy (v1.1 vector: dragged by the boat between fixes)
sl_pinger_preset = sl.declare_arg('pinger_preset', default_value = 'slow_rov') # pinger filter preset (config/pinger_filter.yaml)
sl_pinger_yaml = sl.declare_arg('pinger_filter_yaml', default_value = '~/.config/blueboat_control/pinger_tuning.yaml')  # the LIVE tuning file: created with every filter parameter listed if missing; edit and save while the boat runs, applied within a second ('' = none)

def _pinger_mode():
    mode = str(sl.arg('pinger_mode') or '').strip().lower()
    if mode:
        if mode not in ('off', 'follow', 'log'):
            raise RuntimeError(f"pinger_mode must be off, follow or log, not {mode!r}")
        return mode
    return 'follow' if sl.arg('use_pinger') else 'off'


def launch_setup():
    pinger_mode = _pinger_mode()
    follow = pinger_mode == 'follow'

    # Connect to the robot
    sl.node('mavros',        # package
            'mavros_node',   # executable
            output='screen',
            parameters=[{
                'fcu_url': sl_fcu,
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
                        'pinger_mode': pinger_mode,        # resolved here; wins over the nodes' use_* aliases
                        'data_dir': sl_data,
                        'session_mode': sl_session,
                        'record_bag': sl_bag,
                        'trajectory': sl_trajectory,
                        'use_sim_time': False})

    # Connect to the pinger and log data
    sl.node('blueboat_control', 
            'uwgps_log.py', 
            parameters={'use_sim_time': False})

    # The pinger pipeline (2026-11-24): the ONE publisher of
    # /blueboat/pinger_coordinates and /blueboat/pinger/*, in every real launch
    # (the pinger is tracked whatever the mode; the mode decides following).
    sl.node('blueboat_control',
            'pinger_estimator.py',
            parameters={'pinger_source': sl_pinger_source,
                        'pinger_preset': sl_pinger_preset,
                        'pinger_filter_yaml': sl_pinger_yaml,
                        'use_sim_time': False})

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
                            'pinger_mode': pinger_mode,
                            'has_path': sl_has_path,
                            'data_dir': sl_data,
                            'session_mode': sl_session,
                            'path_source': sl_path_source,
                            'use_sim_time': False})

        # A run session can switch between path and pinger following between
        # runs, so its path server always runs (it watches the station's
        # session trajectory file). Outside a session: not when following the
        # pinger, nor without a path (has_path:=false, pinger mode off or log).
        needs_path = not follow and bool(sl.arg('has_path'))
        if needs_path or sl.arg('session_mode') == True:
                # Compute trajectory and target
                sl.node('blueboat_control', 
                        'path_generation.py', 
                         parameters={'trajectory' : sl_trajectory})


    return sl.launch_description()

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
