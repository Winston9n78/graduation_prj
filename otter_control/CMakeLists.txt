cmake_minimum_required(VERSION 2.8.3)
project(otter_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp tf message_generation std_msgs geometry_msgs sensor_msgs usv_msgs tf2 tf2_ros dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

add_message_files(FILES usv_status.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(controller
  include/otter_control/controller.h
  src/controller.cpp
)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_dependencies(controller ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(param_set
#   include/otter_control/controller.h
#   src/param_server.cpp
#   )
# add_dependencies(param_set ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(param_set ${catkin_LIBRARIES})

