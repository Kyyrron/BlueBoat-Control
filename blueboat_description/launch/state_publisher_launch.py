from simple_launch import SimpleLauncher


sl = SimpleLauncher(use_sim_time = False)

sl.declare_arg('namespace', default_value='blueboat')
sl.declare_arg('jsp', True)
sl.declare_arg('rviz', True)
sl.declare_arg('thr','thrusters_ur')
sl.declare_arg('replay', False)     # kinematic bag replay description (blueboat.xacro)
# Hull and actuator model (blueboat.xacro): identified (default) | bluerov2_legacy
sl.declare_arg('plant', 'identified')
# Who owns heave / roll / pitch of the identified plant (2026-11-15, R-8; blueboat.xacro): sea_node (default,
# the simulator's sea node: CAD hydrostatics, damping, physical CoG) | standalone (Sim_launch.py: box + plugin damping)
sl.declare_arg('vertical_model', 'sea_node')


def launch_setup():
    
    namespace = sl.arg('namespace')
    
    with sl.group(ns=namespace):

        # xacro parsing + change moving joints to fixed if no Gazebo here
        xacro_args = {'namespace': namespace, 'simulation': sl.sim_time, 'thrusters': sl.arg('thr'),
                      'replay': sl.arg('replay'), 'plant': sl.arg('plant'),
                      'vertical_model': sl.arg('vertical_model')}
        sl.robot_state_publisher('blueboat_description', 'blueboat.xacro', xacro_args=xacro_args)

        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)

    with sl.group(if_arg='rviz'):
        sl.rviz(sl.find('blueboat_description', 'rov.rviz'))
        
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
