cmake_minimum_required(VERSION 2.8.3)
project(trackingcam3d_client_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  roscpp
  std_msgs
  nodelet
  camera_info_manager
  dynamic_reconfigure
  pcl_conversions
  #cv_bridge
  sensor_msgs
  stereo_msgs
  tf
)
find_package(OpenCV 3 REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/trackingcam3d_client_ros.cfg
)

include_directories(${catkin_INCLUDE_DIRS})



find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_openni_grabber
#  CATKIN_DEPENDS geometry_msgs image_transport roscpp std_msgs
#  DEPENDS system_lib
)

include(cmake/FindZeroMQ.cmake)
include_directories( ${catkin_INCLUDE_DIRS})
add_subdirectory(src)

add_executable(convert_node src/convert.cpp)
target_link_libraries(convert_node ${catkin_LIBRARIES})

add_executable(laser_scan src/laser_scan/laser_scan.cpp)
target_link_libraries(laser_scan ${catkin_LIBRARIES})
# add_executable(laser_scan src/laser_scan/laser_scan.cpp
#                           src/laser_scan/MyDepthToLaserRectDyn.cpp
#                           src/laser_scan/MyDispToLaserRectDyn.cpp
#                           src/laser_scan/MyPerformanceMeter.cpp)
# target_link_libraries(laser_scan ${catkin_LIBRARIES})

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})
# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
