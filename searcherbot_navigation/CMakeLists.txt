cmake_minimum_required(VERSION 3.0.2)
project(searcherbot_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_core
  roscpp
  roslint
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_core roscpp roslint rospy tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/Navigator.cpp)

set(LINK_LIBS
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES} ${LINK_LIBS})

roslint_cpp(
  src/${PROJECT_NAME}/Navigator.cpp include/${PROJECT_NAME}/Navigator.h
  src/navigation_node.cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS navigation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
