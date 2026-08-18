cmake_minimum_required(VERSION 3.5)
project(blueboat_control)

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/MPC/ur_mpc_control.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/MPC/uvr_mpc_control.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/MPC/ur_mpc.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/MPC/uvr_mpc.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/PID/PID.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/_custom_libraries/path_publisher.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/_custom_libraries/path_generation.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/_custom_libraries/custom_functions.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/_custom_libraries/yaml_trajectory.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/robot_interaction/robot_interface.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/robot_interaction/param_set.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/robot_interaction/uwgps_log.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/master_control.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/simulation_interface.py
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
