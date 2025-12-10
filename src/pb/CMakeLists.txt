cmake_minimum_required(VERSION 3.0.2)
project(pb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  rospack
  tf
  sensor_msgs
  pb_msgs
  geometry_msgs
  visualization_msgs 

  #pcl_ros
)

#find_package(PCL 1.8 REQUIRED)
#find_package(PCL 1.8 REQUIRED COMPONENTS common io segmentation)
#find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#DIRECTORY msg
#add_message_files(
  #FILES
 # point.msg    # x,y
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
   #DEPENDENCIES
   #std_msgs
   #geometry_msgs
   #pb_msgs
 #)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES head
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime message_generation pb_msgs 
CATKIN_DEPENDS rospack
# CATKIN_DEPENDS message_runtime
# CATKIN_DEPENDS geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
# Найдём Eigen3
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Указание путей к библиотекам PCL (иногда требуется явно)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ВЗЯТО отсюда https://wiki.ros.org/navigation/Tutorials/RobotSetup/TF

add_executable(data_node src/data_node.cpp)
target_link_libraries(data_node ${catkin_LIBRARIES} -lwiringPi -lpthread)

add_executable(pos_node src/pos_node.cpp)
target_link_libraries(pos_node ${catkin_LIBRARIES} -lpthread)

add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES} -lpthread)

add_executable(write_node src/write_node.cpp)
target_link_libraries(write_node ${catkin_LIBRARIES} -lpthread)

add_executable(lidar_node src/lidar_node.cpp)
target_link_libraries(lidar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scan_node 
src/scan_node.cpp 
    src/scan_code/pillarScan.cpp # <--- ДОБАВЬ ЭТУ СТРОКУ
    )
target_link_libraries(scan_node 
${catkin_LIBRARIES}
atomic   # <--- ДОБАВИТЬ ЭТУ СТРОКУ
)

#add_executable(calibr_node src/calibr_node.cpp)
#target_link_libraries(calibr_node ${catkin_LIBRARIES} -lpthread)

#add_executable(pillar_node src/pillar_node.cpp)
#target_link_libraries(pillar_node ${catkin_LIBRARIES})

#add_executable(startPose_node src/startPose_node.cpp)
#target_link_libraries(startPose_node ${catkin_LIBRARIES})
