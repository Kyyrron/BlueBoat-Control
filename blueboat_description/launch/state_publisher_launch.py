from simple_launch import SimpleLauncher


sl = SimpleLauncher(use_sim_time = False)

sl.declare_arg('namespace', default_value='blueboat')
sl.declare_arg('jsp', True)
sl.declare_arg('rviz', True)
sl.declare_arg('thr','thrusters_ur')


def launch_setup():
    
    namespace = sl.arg('namespace')
    
    with sl.group(ns=namespace):

        # xacro parsing + change moving joints to fixed if no Gazebo here
        xacro_args = {'namespace': namespace, 'simulation': sl.sim_time, 'thrusters': sl.arg('thr')}
        sl.robot_state_publisher('blueboat_description', 'blueboat.xacro', xacro_args=xacro_args)

        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)

    with sl.group(if_arg='rviz'):
        sl.rviz(sl.find('blueboat_description', 'rov.rviz'))
        
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
