cmake_minimum_required(VERSION 3.0.2)
project(state_machine_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  mav_msgs
  roscpp
  std_msgs
  tf_conversions
  message_generation
)

find_package(Eigen3)

catkin_package(
#  INCLUDE_DIRS include
  # LIBRARIES state_machine_pkg
  CATKIN_DEPENDS
#  DEPENDS system_lib
  eigen_conversions
  geometry_msgs
  mav_msgs
  roscpp
  std_msgs
  tf_conversions
  message_runtime
)

add_library(state_machine_pkg INTERFACE)
target_include_directories(state_machine_pkg INTERFACE include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_machine src/state_machine.cpp)
add_dependencies(state_machine  ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine ${catkin_LIBRARIES})
