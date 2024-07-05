cmake_minimum_required(VERSION 3.1.3)
project(transformer)



find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  pluginlib

)



add_service_files(
  FILES
  CallLLM.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES transformer
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    message_runtime
)





catkin_install_python(PROGRAMS 
                      src/interfaceLLM_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

