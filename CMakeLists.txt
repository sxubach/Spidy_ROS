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

add_message_files(
  FILES
  sensors.msg
) 

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_executable(socket_client_topic_server src/client_socket_topic_PC.cpp)
target_link_libraries(socket_client_topic_server ${catkin_LIBRARIES})

add_executable(test_pwm_client src/test/test_PWMaction_client.cpp)
target_link_libraries(test_pwm_client ${catkin_LIBRARIES})

add_executable(Algorithm_node src/Algorithm_node.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Algorithm_node ${catkin_LIBRARIES})

add_executable(Sinus_node src/Sinus_node.cpp)
target_link_libraries(Sinus_node ${catkin_LIBRARIES})
