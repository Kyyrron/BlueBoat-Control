"""Kinematic replay of a recorded real-robot run in Gazebo (v1.1).

    ros2 launch blueboat_control Replay_launch.py bag:=<run folder or bag dir>
    ros2 launch blueboat_control Replay_launch.py bag:=<...> world_dir:=~/worlds/<path>/<world> rate:=2.0

Starts Gazebo (the stock `ocean` world, or a GPS-anchored world built by the
simulator), spawns the boat with blueboat_description's force-free `replay`
description, bridges /clock and the world's set_pose service, and runs
bag_replay, which moves the boat along the recorded pose and republishes the
recording (see bag_replay.py). Nothing else: no controller, no
simulation_interface, no sea state, no /current bridge, no sonar -- the
recording already holds the real sea, and the boat cannot feel a simulated one.

A built world is only READ here: its world.sdf (with the folder on
GZ_SIM_RESOURCE_PATH, as the simulator's own launch does) and the anchor in its
metadata.yaml (`blueboat_world_meta/1`, the contract the Mission Control
Station reads too). No blueboat_sss_sim code is imported (CM-3).
"""

import math
import os
import re
from pathlib import Path

import yaml
from launch.actions import SetEnvironmentVariable
from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)

sl.declare_arg('bag', default_value = '')               # a run folder (Robot_data/<stem>/) or its *-bag directory
sl.declare_arg('world_dir', default_value = '')         # empty = the stock `ocean` world, in the run's launch frame
sl.declare_arg('rate', default_value = 1.0)             # playback rate (live: /replay/command)
sl.declare_arg('heave', default_value = True)           # replay the EKF's detrended heave; False pins the waterline
sl.declare_arg('start_paused', default_value = False)
sl.declare_arg('gui', default_value = True)

STOCK_WORLD_NAME = 'ocean'


def world_name_from_sdf(sdf_file, default=STOCK_WORLD_NAME):
    try:
        m = re.search(r'<world\s+name\s*=\s*["\']([^"\']+)["\']',
                      Path(sdf_file).read_text(encoding='utf-8'))
    except OSError:
        return default
    return m.group(1) if m else default


def world_anchor(world_dir):
    """(lat0, lon0) of a built world: metadata.yaml geo_anchor, else
    scene_manifest.yaml geo.anchor. None when the folder carries neither."""
    d = Path(world_dir)
    for name, keys in (('metadata.yaml', ('geo_anchor',)),
                       ('scene_manifest.yaml', ('geo', 'anchor'))):
        try:
            doc = yaml.safe_load((d / name).read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            continue
        for k in keys:
            doc = (doc.get(k) if isinstance(doc, dict) else None) or {}
        if isinstance(doc, dict) and 'lat0' in doc and 'lon0' in doc:
            return float(doc['lat0']), float(doc['lon0'])
    return None


def launch_setup():
    bag = os.path.expanduser(str(sl.arg('bag')))
    if not bag:
        raise RuntimeError("launch argument 'bag' is required (a run folder or its bag)")
    world_dir = os.path.expanduser(str(sl.arg('world_dir') or ''))
    gz_flags = '-r' if sl.arg('gui') else '-r -s'

    if world_dir:
        world_file = os.path.join(world_dir, 'world.sdf')
        if not os.path.isfile(world_file):
            raise RuntimeError(f"{world_file}: no such world")
        anchor = world_anchor(world_dir)
        if anchor is None:
            raise RuntimeError(f"{world_dir}: no geo anchor (metadata.yaml geo_anchor)")
        world_name = world_name_from_sdf(world_file)
        # The world references its meshes relatively (seabed.stl).
        sl.add_action(SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join(filter(None, [world_dir, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))))
    else:
        world_file = sl.find('blueboat_description', 'world.sdf')
        anchor = None
        world_name = STOCK_WORLD_NAME

    sl.gz_launch(world_file, gz_flags)
    sl.create_gz_bridge([GazeboBridge.clock()])
    # set_pose: a ROS service bridged onto UserCommands' gz service.
    sl.node('ros_gz_bridge', 'parameter_bridge', name='replay_set_pose_bridge',
            arguments=[f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose'])

    sl.include('blueboat_description', 'upload_rov_launch.py',
               launch_arguments={'sliders': False, 'thr': 'thrusters_ur',
                                 'replay': True, 'gazebo_world_name': world_name})

    sl.node('blueboat_control', 'bag_replay.py', output='screen',
            parameters={'bag': bag,
                        'rate': float(sl.arg('rate')),
                        'world_name': world_name,
                        'anchor_lat': anchor[0] if anchor else math.nan,
                        'anchor_lon': anchor[1] if anchor else math.nan,
                        'heave': sl.arg('heave'),
                        'start_paused': sl.arg('start_paused')})

    return sl.launch_description()


generate_launch_description = sl.launch_description(opaque_function = launch_setup)
