cmake_minimum_required(VERSION 2.8.3)
project(watermellon)

include(MakeICE.cmake)
include(FindJsoncpp.cmake)

##set(ROS_BUILD_TYPE Debug)
##set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib geometry_msgs)
add_action_files(DIRECTORY action FILES 
	wm_navigation_al.action
	)

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
  genmsg
 
)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#message("** CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

FIND_LIBRARY(ice_lib Ice)
FIND_LIBRARY(iceutil_lib IceUtil)
find_package(Jsoncpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

#message("**Jsoncpp_INCLUDE_DIR: " ${JSONCPP_INCLUDE_DIR} )
#message("**Jsoncpp_LIBRARY: " ${JSONCPP_LIBRARY} )

add_message_files(
   FILES
   GNavGoalStamped.msg
)

add_service_files(
  DIRECTORY srv
  FILES GetMap.srv GetObject.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  actionlib_msgs 
  geometry_msgs
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIR}
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
  ${JSONCPP_LIBRARY}
)
##################################################################
# BICA
##################################################################
add_executable(wm_bica_node
	include/wm_bica/BicaIceComms.h src/wm_bica/BicaIceComms.cpp
	include/wm_bica/Component.h src/wm_bica/Component.cpp
	include/wm_bica/Register.h src/wm_bica/Register.cpp
	include/wm_bica/CascadeScheduler.h src/wm_bica/CascadeScheduler.cpp
	include/wm_bica/Singleton.h
	include/wm_bica/BicaDispatcher.h src/wm_bica/BicaDispatcher.cpp
	include/wm_bica/IceComms.h src/wm_bica/IceComms.cpp
	
	src/wm_bica/components/TestSimple.h src/wm_bica/components/TestSimple.cpp	
	src/wm_bica/components/Navigator.h src/wm_bica/components/Navigator.cpp	
	src/wm_navigation/Locations.cpp
	
	src/wm_bica/bica_node.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bica_node bica_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(wm_bica_node
   ${ice_lib}
   ${iceutil_lib}
   ${LINK_LIBS}
)

##################################################################
# Navigation
##################################################################

add_executable(wm_map_server_node 
	src/wm_navigation/wm_map_server_node.cpp
	src/wm_navigation/WmMapServer.cpp
)
add_executable(wm_localization_node 
	src/wm_navigation/wm_localization_node.cpp
	src/wm_navigation/WmLocalization.cpp
)
add_executable(wm_global_navigation_node 
	src/wm_navigation/wm_global_navigation_node.cpp
	src/wm_navigation/WmGlobalNavigationAS.cpp
	src/wm_navigation/WmGlobalNavigation.cpp
	src/wm_navigation/Locations.cpp
)
add_executable(wm_local_navigation_node 
	src/wm_navigation/wm_local_navigation_node.cpp
	src/wm_navigation/WmLocalNavigation.cpp
)
add_executable(wm_test_global_navigation_node 
	src/wm_navigation/wm_test_global_navigation_node.cpp
)
add_executable(wm_fake_localization_node 
	src/wm_navigation/wm_fake_localization_node.cpp
)
add_executable(wm_map_saver 
	src/wm_navigation/wm_map_saver.cpp
)


target_link_libraries(wm_map_server_node  ${LINK_LIBS})
target_link_libraries(wm_localization_node ${LINK_LIBS})
target_link_libraries(wm_local_navigation_node ${LINK_LIBS})
target_link_libraries(wm_global_navigation_node ${LINK_LIBS})
target_link_libraries(wm_test_global_navigation_node ${LINK_LIBS})
target_link_libraries(wm_fake_localization_node ${LINK_LIBS})
target_link_libraries(wm_map_saver ${LINK_LIBS})

add_dependencies(wm_map_server_node watermellon_gencpp)
add_dependencies(wm_global_navigation_node watermellon_gencpp)
add_dependencies(wm_global_navigation_node watermellon_gencpp)
add_dependencies(wm_map_saver watermellon_gencpp)

##################################################################
# Objects
##################################################################

add_executable(wm_object_node 
	src/wm_objects/wm_object_node.cpp
	src/wm_objects/WmObjectTrainer.cpp
)

add_executable(wm_object_saver_node 
	src/wm_objects/wm_object_saver_node.cpp
)

target_link_libraries(wm_object_node  ${LINK_LIBS})
target_link_libraries(wm_object_saver_node  ${LINK_LIBS})

##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
