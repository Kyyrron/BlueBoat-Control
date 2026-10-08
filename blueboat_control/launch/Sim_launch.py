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

        # Launch gazebo and related simulation nodes
        sl.include('blueboat_description',
                   'world_launch.py',
                   launch_arguments={'sliders': False,
                                     'thr': sl_robot,
                                     'yaw': sl_spawn_yaw})

        # Simulation interaction
        sl.node('blueboat_control',
                'simulation_interface.py',
                parameters={'controller_type' : sl_controller,
                            'note': sl_note,
                            'data_dir': sl_data})

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
                parameters={'controller_type' : sl_controller,
                            'simulation' : True,
                            'data_dir': sl_data})

        return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function = launch_setup)
