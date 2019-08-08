
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

project(walk_stablization_controller)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  choreonoid
  std_msgs
  sensor_msgs
  trajectory_msgs
  #iir
)


#catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

#set(iir_DIR=/usr/local/lib/cmake/iir)

add_definitions(${CHOREONOID_DEFINITIONS})
FIND_PACKAGE(Choreonoid REQUIRED)
FIND_PACKAGE(iir REQUIRED)

ADD_DEFINITIONS(${CHOREONOID_DEFINITIONS})

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  #/usr/local/include/acado
)

LINK_DIRECTORIES(${CHOREONOID_LIBRARY_DIRS})

function(add_cnoid_simple_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES} iir::iir)
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)
endfunction()

add_subdirectory(src)

