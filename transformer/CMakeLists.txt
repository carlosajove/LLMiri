cmake_minimum_required(VERSION 3.1.3)
project(transformer)



find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  pluginlib
  geometry_msgs
)

add_message_files(
    FILES
    CallLLMlist.msg 
    CallLLMsingle.msg
)


add_service_files(
  FILES
  CallLLM.srv
  CallLLM_long.srv
  GetObjectPoseFromDescription.srv
  GetGripperForceFromDescription.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES transformer
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs
    message_runtime
)





catkin_install_python(PROGRAMS 
                      src/rosmelodic/llm_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

