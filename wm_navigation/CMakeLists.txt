cmake_minimum_required(VERSION 2.8.3)
project(wm_navigation)

##set(ROS_BUILD_TYPE Debug)
##set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  octomap_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
  dynamic_reconfigure
  nodelet
  message_generation
  costmap_2d
  navfn
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

add_service_files(
  DIRECTORY srv
  FILES GetMap.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

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

#add_library(${PROJECT_NAME} src/WmMapServer.cpp src/WmLocalization.cpp src/WmGlobalNavigation.cpp )
#target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(wm_map_server_node 
	src/wm_map_server_node.cpp
	src/WmMapServer.cpp
)
target_link_libraries(wm_map_server_node 
	${LINK_LIBS}
)

add_executable(wm_localization_node 
	src/wm_localization_node.cpp
	src/WmLocalization.cpp
)

target_link_libraries(wm_localization_node 
	${LINK_LIBS}
)

add_executable(wm_global_navigation_node 
	src/wm_global_navigation_node.cpp
	src/WmGlobalNavigation.cpp
)
add_executable(wm_local_navigation_node 
	src/wm_local_navigation_node.cpp
	src/WmLocalNavigation.cpp
)
target_link_libraries(wm_local_navigation_node 
	${LINK_LIBS}
)

target_link_libraries(wm_global_navigation_node 
	${LINK_LIBS}
)

add_executable(wm_fake_localization_node 
	src/wm_fake_localization_node.cpp
)

target_link_libraries(wm_fake_localization_node 
	${LINK_LIBS}
)

add_executable(wm_map_saver 
	src/wm_map_saver.cpp
)

target_link_libraries(wm_map_saver 
	${LINK_LIBS}
)

##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
