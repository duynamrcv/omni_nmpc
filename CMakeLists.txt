cmake_minimum_required(VERSION 3.0.2)
project(omni_nmpc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_package()

include_directories(
  include
  include/eigen3
  acado_mpc_export
  ${catkin_INCLUDE_DIRS}
)

add_library(ACADO STATIC IMPORTED)

add_executable(nmpc_node src/nmpc_node.cpp src/acado.cpp)
set_target_properties(ACADO PROPERTIES IMPORTED_LOCATION
                      "${CMAKE_CURRENT_SOURCE_DIR}/acado_mpc_export/libacado_exported_rti.a")
target_link_libraries(nmpc_node ${catkin_LIBRARIES} ACADO)
