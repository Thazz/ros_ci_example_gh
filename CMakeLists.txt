cmake_minimum_required(VERSION 2.8.3)
project(ros_ci_example_gh)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY 
   action
  FILES
   Quit.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    roscpp
    message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gh_quit src/quit_after_x_sec.cpp)
add_dependencies(gh_quit ${catkin_EXPORTED_TARGETS} smartgv_chatter_gencpp)
target_link_libraries(gh_quit ${catkin_LIBRARIES})