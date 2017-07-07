cmake_minimum_required(VERSION 2.8.3)
project(mapserver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rosapi
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  mapserver_msgs
  std_msgs
  geometry_msgs
  rosconsole
  rosgraph_msgs
  roslang
  rostime
  tf
  tf_conversions
  genmsg
  laser_geometry
  message_generation
  mrpt_rawlog
  mrpt_map
  mrpt_icp_slam_2d
  mrpt_rbpf_slam
  mrpt_msgs
  mrpt_bridge
  mrpt_reactivenav2d
  mrpt_graphslam_2d
  mrpt_ekf_slam_2d
  mrpt_localization
  mrpt_tutorials
  mrpt_ekf_slam_3d
)


set(MRPT_DONT_USE_DBG_LIBS TRUE)
find_package(MRPT REQUIRED COMPONENTS base maps obs slam gui)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.1 REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  rsm.srv
  ism.srv
  ismStackFloat.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  mapserver_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapserver
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Declare a C++ library
# add_library(mapserver
#   src/${PROJECT_NAME}/mapserver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(mapserver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(mapserver_node src/mapserver_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(mapserver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(mapserver
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mapserver mapserver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mapserver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(mapserver src/mapserver.cpp)
target_link_libraries(mapserver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(mapserver_raw src/mapserver_raw.cpp)
target_link_libraries(mapserver_raw
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(mapserver_raw_node src/mapserver_raw_node.cpp)
target_link_libraries(mapserver_raw_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  mapserver_raw
)

add_executable(tile_publisher src/tile_publisher.cpp)
target_link_libraries(tile_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(mapserver_raw_rpc_test src/mapserver_raw_rpc_test.cpp)
target_link_libraries(mapserver_raw_rpc_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(mapserver_stat_rpc_mapStack_test src/mapserver_stat_rpc_mapStack_test.cpp)
target_link_libraries(mapserver_stat_rpc_mapStack_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(mapserver_stat src/mapserver_stat.cpp)
target_link_libraries(mapserver_stat
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${MRPT_LIBRARIES}
)

add_executable(mapserver_stat_node src/mapserver_stat_node.cpp)
target_link_libraries(mapserver_stat_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${MRPT_LIBRARIES}
  mapserver_stat
)

add_executable(mapserver_mapstorage_trigger_node src/mapserver_storage_trigger.cpp)
target_link_libraries(mapserver_mapstorage_trigger_node
  ${catkin_LIBRARIES}
)

# Add the msg and srv generation as dependency
add_dependencies(tile_publisher mapserver_gencpp)
add_dependencies(mapserver_raw mapserver_gencpp)
add_dependencies(mapserver_raw_node mapserver_gencpp)
add_dependencies(mapserver_raw_rpc_test mapserver_msgs_gencpp mapserver_gencpp)
add_dependencies(mapserver_stat mapserver_msgs_gencpp mapserver_gencpp)
add_dependencies(mapserver_stat_node mapserver_msgs_gencpp mapserver_gencpp)
add_dependencies(mapserver_stat_rpc_mapStack_test mapserver_msgs_gencpp mapserver_gencpp)

add_definitions("-std=c++11")



catkin_package(
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
#   LIBRARIES
  CATKIN_DEPENDS
    nav_msgs
    rosapi
    roscpp
    rospy
    nav_msgs
    sensor_msgs
    mapserver_msgs
    std_msgs
    geometry_msgs
    rosconsole
    rosgraph_msgs
    roslang
    rostime
    tf
    tf_conversions
    genmsg
    laser_geometry
    message_generation
    mrpt_rawlog
    mrpt_map
    mrpt_icp_slam_2d
    mrpt_rbpf_slam
    mrpt_msgs
    mrpt_bridge
    mrpt_reactivenav2d
    mrpt_graphslam_2d
    mrpt_ekf_slam_2d
    mrpt_localization
    mrpt_tutorials
    mrpt_ekf_slam_3d
  DEPENDS
    ${EIGEN_PACKAGE}
    ${OpenCV_PACKAGE}
    ${Boost_PACKAGE}
)