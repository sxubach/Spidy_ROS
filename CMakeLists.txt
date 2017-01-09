cmake_minimum_required(VERSION 2.8.3)
project(communication_pkg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  genmsg
  roscpp
  rospy
  std_msgs
)

add_action_files(
DIRECTORY
  action
FILES
  PWM.action
)

generate_messages(
DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(socket_client_topic_server src/client_socket_topic_PC.cpp)
target_link_libraries(socket_client_topic_server ${catkin_LIBRARIES})

add_executable(test_pwm_client src/test_PWMaction_client.cpp)
target_link_libraries(test_pwm_client ${catkin_LIBRARIES})


