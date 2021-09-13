cmake_minimum_required(VERSION 3.0.2)
project(searcherbot_examples)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  geometry_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_bt
  CATKIN_DEPENDS behaviortree_cpp_v3 geometry_msgs roscpp roslint sensor_msgs std_msgs
)

add_library(${PROJECT_NAME}_bt SHARED
  src/${PROJECT_NAME}/bt/MoveBaseActions.cpp
  src/${PROJECT_NAME}/bt/ScanAction.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${PROJECT_NAME}_bt
  ${catkin_LIBRARIES}
)

add_executable(bumpgo src/bumpgo.cpp)
target_link_libraries(bumpgo ${LINK_LIBS})

roslint_cpp(
  src/${PROJECT_NAME}/bt/MoveBaseActions.cpp include/${PROJECT_NAME}/bt/MoveBaseActions.h
  src/${PROJECT_NAME}/bt/ScanAction.cpp include/${PROJECT_NAME}/bt/ScanAction.h
  src/bumpgo.cpp
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}_bt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS bumpgo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)