cmake_minimum_required(VERSION 3.0.2)
project(data)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  tf
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES

  Struct_Car.msg
  Struct_Control.msg
  Struct_Encoder.msg
  Struct_IMU.msg
  Struct_Led.msg
  Struct_Odom.msg
  Struct_Sensor.msg
  Struct_Servo.msg
  Struct_ServoOut.msg
  Struct_Joy.msg
  
  Struct_Data2Driver.msg
  Struct_Driver2Data.msg
  
  Rpy.msg
  Bme.msg
  Command.msg
  Odomet.msg
  Power.msg
  Control.msg
  DataTimeMy.msg
  data_driver_info.msg
  data_driver_control.msg
  data_driver_odometr.msg
  data_iot_info.msg
  data_iot_distance.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES head
#  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ../../devel/include
  ../../src/data/src
  ../../src/data/msg
  ../../src/data/include
  ../../src/data/include/data
  /usr/include/
)

add_executable(data_node src/data_node.cpp)
target_link_libraries(data_node ${catkin_LIBRARIES} -lwiringPi -lpthread)

add_executable(head_node src/head_node.cpp)
target_link_libraries(head_node ${catkin_LIBRARIES})