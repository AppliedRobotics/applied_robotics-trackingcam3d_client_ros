find_package(OpenCV 3 REQUIRED)
#find_package(ZeroMQ 4 REQUIRED)
find_package(libtconfig REQUIRED)
find_package(libtlive REQUIRED)
#find_package(libt3dproc REQUIRED)

add_executable(trackingcam3d_client_ros	main.cpp 					
		MyDepthIllustrator.cpp
		MyFpsMeter.cpp)
                    
# make sure configure headers are built before any node using them
add_dependencies(trackingcam3d_client_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(trackingcam3d_client_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} tbb ${LIBTCONFIG_LIBRARIES} ${LIBTLIVE_LIBRARIES} ${LIBT3DPROC_LIBRARIES})
target_link_libraries(trackingcam3d_client_ros ${ZeroMQ_LIBRARY})

## Mark executables and/or libraries for installation
install(TARGETS trackingcam3d_client_ros DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
