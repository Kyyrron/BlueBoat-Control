from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():
    
    sl = SimpleLauncher()
    sl.declare_arg('gui', default_value=True)
    sl.declare_arg('spawn', default_value=True)
    sl.declare_arg('thr',default_value = 'thrusters_ur')
    sl.declare_arg('spawn_pose', default_value = "0.0 0.0 0.0 0.0 0.0 0.0")

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
                   launch_arguments={'thr': sl.arg('thr'), 'spawn_pose': sl.arg('spawn_pose')})
    return sl.launch_description()


"""
from simple_launch import SimpleLauncher, GazeboBridge
import os

def generate_launch_description():
    
    sl = SimpleLauncher()
    sl.declare_arg('gui', default_value=True)
    sl.declare_arg('spawn', default_value=True)
    sl.declare_arg('thr',default_value = 'thrusters_ur')
    sl.declare_arg('world',default_value = 'world.sdf')
    sl.declare_arg('spawn_pose', default_value = "0.0 0.0 0.0 0.0 0.0 0.0")

    world = sl.arg('world')                 
    if not world:
        world = sl.find('blueboat_description', 'world.sdf')
    else:
        world = sl.find('blueboat_description', world)

    with sl.group(if_arg='gui'):
        sl.gz_launch(world, '-r')
    with sl.group(unless_arg='gui'):
        sl.gz_launch(world, '-r -s')

    bridges = [GazeboBridge.clock(),
               GazeboBridge('/ocean_current', '/current', 'geometry_msgs/Vector3',
                            GazeboBridge.ros2gz)]
        
    sl.create_gz_bridge(bridges)

    with sl.group(if_arg='spawn'):
        sl.include('blueboat_description', 'upload_rov_launch.py',
                   launch_arguments={'thr': sl.arg('thr'), 'spawn_pose': sl.arg('spawn_pose')})
    return sl.launch_description()

"""