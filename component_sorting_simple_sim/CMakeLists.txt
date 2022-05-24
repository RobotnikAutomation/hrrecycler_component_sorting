cmake_minimum_required(VERSION 2.8.3)
project(component_sorting_simple_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(box_spawner_node src/box_spawner.cpp)

add_dependencies(box_spawner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(box_spawner_node
${catkin_LIBRARIES}
)

install(DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
