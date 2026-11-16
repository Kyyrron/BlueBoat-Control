from pathlib import Path

import yaml
from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)

sl_robot = sl.declare_arg('robot_file', default_value='thrusters_ur')           # Choose between 2 and 3 thrusters architecture (not functionnal yet)
sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping') # Trajectory reference for the control
sl_controller = sl.declare_arg('controller_type', default_value = 'MPC')        # Controller to be used
sl_data = sl.declare_arg('data_dir', default_value = '')                        # Root for the position CSV and the controller .npy log. Empty resolves to $BLUEBOAT_DATA_DIR, else the sourced workspace root.
sl_note = sl.declare_arg('note', default_value = 'sim')                         # Goes into the position-log file name, exactly as on the real boat.
sl_spawn_yaw = sl.declare_arg('spawn_yaw', default_value = 0.)                  # Boat spawn heading, RADIANS ENU (0 = East). Spawn stays at (0,0).
sl_path_source = sl.declare_arg('path_source', default_value = 'mission')       # 'mission' = follow `trajectory` (today's behaviour). 'replanner' = live path update: hold at the spawn pose until /blueboat/update_path loads the first path (docs/PATH_UPDATE_INTERFACE.md).

# --- which simulation (2026-11-13; docs/PARAMETER_REGISTER.md, FIELD_TUNING.md) ---------------
# sim_config = field_equivalent (default): the identified BlueBoat plant, the real boat's gains / MPC
#              model / horizon / weights, thrust applied at 20 Hz and /blueboat/odom at the field rate
#              and delay. What a simulated result that must transfer to the boat is run with.
#            = legacy: the BlueROV2 plant, the sim-tuned gains, 10 Hz thrust, Gazebo's clean 20 Hz
#              odometry - every simulation made before 2026-11-13, reproduced.
# The four arguments below override single pieces of the chosen configuration ('' = keep it).
sl_sim_config = sl.declare_arg('sim_config', default_value = 'field_equivalent')
sl_plant = sl.declare_arg('plant', default_value = '')                 # identified | bluerov2_legacy
sl_odom_profile = sl.declare_arg('odom_profile', default_value = '')   # passthrough | field_3hz | field_4hz | field_13hz
sl_odom_seed = sl.declare_arg('odom_seed', default_value = 0)          # seed of the odometry timing draws
sl_vertical_model = sl.declare_arg('vertical_model', default_value = 'standalone')  # identified plant (R-8, 2026-11-15): standalone = this launch starts no sea node, so Gazebo's box + plugin damping + box CoG | sea_node = a sea node is started beside it (CAD hydrostatics, its damping, physical CoG)
sl_tuning_file = sl.declare_arg('tuning_file', default_value = '')     # ROS parameter file of gains flown on a given day (config/field_runs/*.yaml)

SIM_CONFIGS = {
        'field_equivalent': dict(plant='identified', odom_raw=True, gain_profile='field',
                                 thrust_rate_hz=20.0, odom_profile='field_13hz'),
        'legacy':           dict(plant='bluerov2_legacy', odom_raw=False, gain_profile='sim',
                                 thrust_rate_hz=10.0, odom_profile='passthrough'),
}


def _path_window(trajectory):
        """path_publisher's `total_time` for a from_yaml mission, else None.

        path_publisher requests one fixed window, built once at startup (1000 s
        by default), so RViz drew only the start of any longer mission. The
        window is sized from the trajectory's duration x 10 % + 30 s, the rule
        of blueboat_sss_sim's full_mission_launch.py. An MCS deployed file
        (.deployed/<name>.yaml) does not exist yet at launch -- MCS writes it
        once its GPS fit converges -- so the designer file it is deployed from,
        <trajectories>/<name>.yaml, is read first. Built-in shapes carry no
        duration: None keeps path_publisher's default.
        """
        if not str(trajectory).startswith('from_yaml'):
                return None
        raw = str(trajectory).partition(':')[2]
        if not raw:
                return None
        f = Path(raw).expanduser()
        candidates = [f]
        if f.parent.name == '.deployed':
                candidates.insert(0, f.parent.parent / f.name)
        for cand in candidates:
                try:
                        doc = yaml.safe_load(cand.read_text(encoding='utf-8')) or {}
                        duration = float(doc.get('duration_s') or 0.0)
                        if duration <= 0.0 and doc.get('points'):
                                duration = float(doc['points'][-1][0])
                except Exception:
                        continue
                if duration > 0.0:
                        return duration * 1.1 + 30.0
        return None


def launch_setup():

        name = str(sl.arg('sim_config'))
        if name not in SIM_CONFIGS:
                raise RuntimeError(f"sim_config:={name}: expected {' | '.join(SIM_CONFIGS)}")
        cfg = dict(SIM_CONFIGS[name])
        if str(sl.arg('plant')):
                cfg['plant'] = str(sl.arg('plant'))
        tuning_file = str(sl.arg('tuning_file'))
        odom_profile = {'odom_profile': cfg['odom_profile']}
        if str(sl.arg('odom_profile')):
                odom_profile = {'odom_profile': str(sl.arg('odom_profile'))}
        elif tuning_file:
                # a field-run file states the odometry timing of its day
                odom_profile = {}
        if tuning_file and not Path(tuning_file).expanduser().is_file():
                raise RuntimeError(f"tuning_file:={tuning_file}: no such file")

        def with_tuning(parameters):
                # The file first, the launch's own values last: controller_type,
                # simulation, data_dir, path_source and time_source always win.
                return [str(Path(tuning_file).expanduser()), parameters] if tuning_file else parameters

        # Launch gazebo and related simulation nodes
        sl.include('blueboat_description',
                   'world_launch.py',
                   launch_arguments={'sliders': False,
                                     'thr': sl_robot,
                                     'yaw': sl_spawn_yaw,
                                     'plant': cfg['plant'],
                                     'odom_raw': cfg['odom_raw'],
                                     'vertical_model': sl_vertical_model})

        # Simulation interaction
        sl.node('blueboat_control',
                'simulation_interface.py',
                parameters=with_tuning({'controller_type' : sl_controller,
                            'note': sl_note,
                            'data_dir': sl_data,
                            # sim-time watchdog / republish: one clock at
                            # any Gazebo real-time factor (2026-11-07)
                            'time_source': 'ros',
                            'thrust_rate_hz': cfg['thrust_rate_hz'],
                            'odom_seed': int(sl.arg('odom_seed')),
                            **odom_profile}))

        # Compute trajectory and target
        sl.node('blueboat_control',
                'path_generation.py',
                parameters={'trajectory' : sl_trajectory})

        # Display trajectory and target in rviz -- the whole mission
        window = _path_window(sl.arg('trajectory'))
        if window is not None:
                sl.node('blueboat_control',
                        'path_publisher.py',
                        parameters={'total_time': window})
        else:
                sl.node('blueboat_control',
                        'path_publisher.py')

        # Load controller
        sl.node('blueboat_control',
                'master_control.py',
                parameters=with_tuning({'controller_type' : sl_controller,
                            'simulation' : True,
                            'data_dir': sl_data,
                            'path_source': sl_path_source,
                            # .npy / monitoring stamps, brakes and path
                            # timeouts on /clock (2026-11-07)
                            'time_source': 'ros',
                            'gain_profile': cfg['gain_profile']}))

        return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function = launch_setup)
