cmake_minimum_required(VERSION 3.1.3)
project(hl_control)



find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  pluginlib
  dynamic_reconfigure
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
    dynamic_reconfigure
)





catkin_install_python(PROGRAMS 
                      src/LLM_to_cartesian.py
                      src/launch_nodes.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
