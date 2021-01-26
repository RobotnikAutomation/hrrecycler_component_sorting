cmake_minimum_required(VERSION 3.0.2)
project(component_sorting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  component_sorting_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rcomponent
  roscpp
  ur_msgs
  moveit_visual_tools
  rviz_visual_tools
  gazebo_ros_link_attacher
)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(component_sorting
 src/object.cpp

)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(component_sorting_node src/component_sorting_node.cpp src/component_sorting.cpp)
add_executable(publish_collision_object_node src/publish_collision_object.cpp)




add_dependencies(component_sorting_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publish_collision_object_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(component_sorting_node
  ${catkin_LIBRARIES}
  component_sorting
)

target_link_libraries(publish_collision_object_node
  ${catkin_LIBRARIES}
)



install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
