cmake_minimum_required(VERSION 3.1.3)
project(ll_control)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  controller_interface
  eigen_conversions
  franka_hw
  hardware_interface
  pluginlib
  gazebo_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_service_files(
  FILES
  SetPose.srv
  GetPose.srv
  GetObjectPose.srv
  SetDmp.srv
  SetInt.srv 
  SetFloat.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ll_control
  CATKIN_DEPENDS 
    roscpp
    rospy 
    geometry_msgs 
    std_msgs 
    message_runtime
    controller_interface
    eigen_conversions
    franka_hw
    hardware_interface
    pluginlib
    DEPENDS Franka
)


add_library(ll_control
  src/read_only_controller_Node.cpp
)


add_dependencies(ll_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(ll_control PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(ll_control SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(ll_control PUBLIC
  include
)

## Installation
install(TARGETS ll_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ll_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





catkin_install_python(PROGRAMS 
                      src/DesiredCartesianPublisher.py
                      src/testing/dmp_test.py 
                      src/my_dmpbbo/my_dmpbbo.py 
                      src/utils/utils.py
                      src/EndEffTrajectoryManager.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



install(DIRECTORY src/testing/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/testing
)

install(DIRECTORY src/my_dmpbbo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/my_dmpbbo
)




## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(ll_control FILES ${SOURCES} ${HEADERS})
  add_tidy_target(ll_control
    FILES ${SOURCES}
    DEPENDS ll_control
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()