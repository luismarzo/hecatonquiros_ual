cmake_minimum_required(VERSION 2.8.3)
project(heca_plugins)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  message_generation
)



add_service_files(
   FILES
   AddTwoInts.srv
   Set_activation.srv
   Set_position.srv
 )


generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
  LIBRARIES ${PROJECT_NAME}
)



add_executable(int_server src/int_server.cpp)
target_link_libraries(int_server ${catkin_LIBRARIES})
add_dependencies(int_server heca_plugins_gencpp)

add_executable(int_client src/int_client.cpp)
target_link_libraries(int_client ${catkin_LIBRARIES})
add_dependencies(int_client heca_plugins_gencpp)

add_library(positioner_plugin SHARED src/positioner_plugin.cpp)  #los plugins no tienen ejecutables loco
target_link_libraries(positioner_plugin ${GAZEBO_LIBRARIES})

add_executable(Set_activation_client src/Set_activation_client.cpp)
target_link_libraries(Set_activation_client ${catkin_LIBRARIES})
add_dependencies(Set_activation_client heca_plugin_gencpp)

