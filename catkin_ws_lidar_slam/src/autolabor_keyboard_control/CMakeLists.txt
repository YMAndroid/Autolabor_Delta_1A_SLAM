cmake_minimum_required(VERSION 2.8.3)
project(autolabor_keyboard_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_control_node src/keyboard_control_node.cpp)

add_dependencies(keyboard_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(keyboard_control_node
  ${catkin_LIBRARIES}
)
