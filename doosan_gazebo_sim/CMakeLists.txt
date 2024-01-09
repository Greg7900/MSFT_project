cmake_minimum_required(VERSION 3.0.2)
project(doosan_gazebo_sim)

#Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  moveit_ros_planning_interface 
  moveit_ros_move_group
  moveit_core
  tf
  # image_transport
  # cv_bridge
  # sensor_msgs
  # visp_ros
)
find_package(gazebo REQUIRED)
# find_package(VISP REQUIRED)


catkin_package(
  INCLUDE_DIRS 
    include
  
  CATKIN_DEPENDS 
    roscpp 
    # image_transport 
    # cv_bridge 
    # sensor_msgs
    # visp_ros
  DEPENDS 
    gazebo_ros 
    # VISP 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR}
  # ${VISP_INCLUDE_DIRS} 

)
add_executable(gazebo_cube
  src/gazebo_cube.cpp
  src/gazebo_cube_node.cpp
)
target_link_libraries(gazebo_cube
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
  # ${VISP_LIBRARIES} 
)

# add_executable(orientation
#   src/orientation.cpp
# )
# add_executable(visp_test
#   src/visp_test.cpp
# )
# target_link_libraries(visp_test
#   ${catkin_LIBRARIES} 
#   ${VISP_LIBRARIES} 
#   ${GAZEBO_LIBRARIES}
# )
# target_link_libraries(orientation
#   ${catkin_LIBRARIES} 
#   ${VISP_LIBRARIES} 
#   ${GAZEBO_LIBRARIES}
# )


