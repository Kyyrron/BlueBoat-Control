cmake_minimum_required(VERSION 3.5)
project(blueboat_control)

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Controller configurations (gains flown on a given field day) and the pinger
# filter parameters with the station's tuning pictures.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Every node and every library lands FLAT in lib/blueboat_control: they import
# each other as bare modules (`import frame_math`), so a file missing here works
# from the source tree and dies with ModuleNotFoundError after a clean build.
# poslog_report.py is also the post-mission report CLI
# (`ros2 run blueboat_control poslog_report.py <csv>`).
install(PROGRAMS
        src/master_control.py
        src/simulation_interface.py
        src/bag_replay.py
        src/robot_interaction/robot_interface.py
        src/robot_interaction/param_set.py
        src/robot_interaction/uwgps_log.py
        src/robot_interaction/pinger_estimator.py
        src/MPC/ur_mpc.py
        src/PID/PID.py
        src/_custom_libraries/path_publisher.py
        src/_custom_libraries/path_generation.py
        src/_custom_libraries/custom_functions.py
        src/_custom_libraries/yaml_trajectory.py
        src/_custom_libraries/frame_math.py
        src/_custom_libraries/robot_log_schema.py
        src/_custom_libraries/poslog_report.py
        src/_custom_libraries/path_stamp.py
        src/_custom_libraries/thrust_limits.py
        src/_custom_libraries/run_bag.py
        src/_custom_libraries/path_update.py
        src/_custom_libraries/path_update_log.py
        src/_custom_libraries/replay_attitude.py
        src/_custom_libraries/odom_emulation.py
        src/_custom_libraries/pinger_filter.py
        src/_custom_libraries/pinger_params.py
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
