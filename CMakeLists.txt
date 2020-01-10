cmake_minimum_required(VERSION 2.8.3)
project(ros_ci_example_bb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(quit_after_x_sec src/quit_after_x_sec.cpp)
target_link_libraries(quit_after_x_sec ${catkin_LIBRARIES})