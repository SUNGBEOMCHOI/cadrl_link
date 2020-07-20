cmake_minimum_required(VERSION 3.0.3)

project(link_navigation)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
  move_base
  link_waypoint_nav
  robot_localization
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_navigation
#  CATKIN_DEPENDS move_base link_waypoint_nav robot_localization
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

