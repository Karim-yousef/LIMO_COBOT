cmake_minimum_required(VERSION 3.0.2)
project(limo_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  message_filters
  message_generation
)

# Generate services
add_service_files(
  FILES
  MissionTrigger.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/blue_cube_approach_pick_enhanced.py
  scripts/final_arm_pick.py
  scripts/mission_controller.py
  scripts/red_basket_approach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
