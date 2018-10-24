cmake_minimum_required(VERSION 2.8.3)
project(tsp_mtsp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uav_abstraction_layer
)
 

catkin_package(

)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)





add_compile_options(-std=c++11)


#add_executable(points_and_lines src/points_and_lines.cpp)
#target_link_libraries(points_and_lines ${catkin_LIBRARIES})

#add_executable(points_and_lines_2 src/points_and_lines_2.cpp)
#target_link_libraries(points_and_lines_2 ${catkin_LIBRARIES})

#add_executable(points_and_lines_3 src/points_and_lines_3.cpp)
#target_link_libraries(points_and_lines_3 ${catkin_LIBRARIES})

#add_executable(points_and_lines_4 src/points_and_lines_4.cpp)
#target_link_libraries(points_and_lines_4 ${catkin_LIBRARIES})

#add_executable(points_and_lines_5 src/points_and_lines_5.cpp)
#target_link_libraries(points_and_lines_5 ${catkin_LIBRARIES})

#add_executable(points_and_lines_6 src/points_and_lines_6.cpp)
#target_link_libraries(points_and_lines_6 ${catkin_LIBRARIES})
  
#add_executable(tsp_script src/tsp_script.cpp)
#target_link_libraries(tsp_script ${catkin_LIBRARIES})
 
#add_executable(tsp_script_start src/tsp_script_start.cpp)
#target_link_libraries(tsp_script_start ${catkin_LIBRARIES})

#add_executable(tsp_script_sucio src/tsp_script_sucio.cpp)
#target_link_libraries(tsp_script_sucio ${catkin_LIBRARIES})
 
#add_executable(tsp_clases src/tsp_clases.cpp)
#target_link_libraries(tsp_clases ${catkin_LIBRARIES})
   
#add_executable(mtsp_script_1 src/mtsp_script_1.cpp)
#target_link_libraries(mtsp_script_1 ${catkin_LIBRARIES})


file(GLOB_RECURSE COMMON_FILES "src/mtsp_files/clases/*.cpp" "src/mtsp_files/clases/*.h" "src/mtsp_files/*.cpp" "src/mtsp_files/*.h")

#add_executable(tsp_script_start_2 src/tsp_script_start_2.cpp )
#target_link_libraries(tsp_script_start_2 ${catkin_LIBRARIES})

#add_executable(tsp_script_start src/tsp_script_start.cpp ${COMMON_FILES})
#target_link_libraries(tsp_script_start ${catkin_LIBRARIES})



add_executable(tsp_start_limpio src/tsp_start_limpio.cpp ${COMMON_FILES})
target_link_libraries(tsp_start_limpio ${catkin_LIBRARIES})

add_executable(test_ual_interfaces_mtsp src/test_ual_interfaces_mtsp.cpp ${COMMON_FILES})
target_link_libraries(test_ual_interfaces_mtsp ${catkin_LIBRARIES})

#add_executable(matriz_c src/matriz_c.cpp)
#target_link_libraries(matriz_c ${catkin_LIBRARIES})
