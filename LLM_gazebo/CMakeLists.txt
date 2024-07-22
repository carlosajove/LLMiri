cmake_minimum_required(VERSION 3.1.3)
project(llm_gazebo)



find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_dev
  gazebo_ros_control
  std_msgs
  message_generation
)

add_message_files(
    FILES
    NameDescriptionPair.msg 
    ObjectDescriptionList.msg
)

#add_service_files(
#  FILES
#  CallLLM.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS    
    rospy
    gazebo_ros_control
    message_runtime
    std_msgs
)





catkin_install_python(PROGRAMS 
                      src/object_description_publisher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

