cmake_minimum_required(VERSION 3.0.2)
project(state_machine_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  roslaunch
  visualization_msgs
)

find_package(Eigen3)

catkin_package(
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    mav_msgs
    roscpp
    std_msgs
    tf_conversions
    roslaunch
    roslib
    visualization_msgs
)

#add_service_files(
#  FILES
#  Transition.srv
#)

add_library(state_machine_pkg INTERFACE)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_machine src/state_machine.cpp)
add_dependencies(state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine ${catkin_LIBRARIES})

#add_executable(state_transition src/state_transition.cpp)
#add_dependencies(state_transition ${catkin_EXPORTED_TARGETS})
#target_link_libraries(state_transition ${catkin_LIBRARIES})


