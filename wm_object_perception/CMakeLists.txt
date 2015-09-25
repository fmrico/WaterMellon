cmake_minimum_required(VERSION 2.8.3)
project(wm_object_perception)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  geometry_msgs
  interactive_markers
  openni2_camera
  pcl_ros
  roscpp
  rospy
  tf
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS PCL
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(wm_modeler 
	src/wm_modeler.cpp 
	src/wm_imarkers.cpp 
)
target_link_libraries(wm_modeler
   ${LINK_LIBS} 
)
