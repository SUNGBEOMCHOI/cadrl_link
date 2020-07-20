cmake_minimum_required(VERSION 3.0.3)
project(link_waypoint_nav)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
  depthimage_to_laserscan
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_navigation
#  CATKIN_DEPENDS move_base link_navigation robot_localization
#  DEPENDS system_lib
)

roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_waypoint src/gps_waypoint.cpp)
add_executable(gps_waypoint_mapping src/gps_waypoint_mapping.cpp)
add_executable(calibrate_heading src/calibrate_heading.cpp)
add_executable(gps_broadcaster src/gps_broadcaster.cpp)
add_executable(link_collect_gps src/link_collect_gps.cpp)
add_executable(point_collect_sim src/point_collect_sim.cpp)
add_executable(point_collect_latlong src/point_collect_to_latlong.cpp)
add_executable(send_waypoint_sim src/send_waypoint_sim.cpp)

target_link_libraries(gps_waypoint ${catkin_LIBRARIES})
target_link_libraries(gps_waypoint_mapping ${catkin_LIBRARIES})
target_link_libraries(calibrate_heading ${catkin_LIBRARIES})
target_link_libraries(gps_broadcaster ${catkin_LIBRARIES})
target_link_libraries(link_collect_gps ${catkin_LIBRARIES})
target_link_libraries(point_collect_sim ${catkin_LIBRARIES})
target_link_libraries(point_collect_latlong ${catkin_LIBRARIES})
target_link_libraries(send_waypoint_sim ${catkin_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





