# - Config file for SyboTech Tracking Camera SDK
# It defines the following variables:
#    LIBTCONFIG_INCLUDE_DIRS - Include directories
#    LIBTCONFIG_LIBRARIES    - link libraries

get_filename_component(LIBTCONFIG_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(LIBTCONFIG_ROOT "${LIBTCONFIG_CMAKE_DIR}/../../../")

if(NOT TARGET tconfig)
	include("${LIBTCONFIG_CMAKE_DIR}/libtconfig-targets.cmake")
endif()

# For the case when we use LIBTCONFIG from build directory, 
# and path to LIBTCONFIG-config.cmake was taken from cmake cache
set(LIBTCONFIG_INCLUDE_DIRS "${LIBTCONFIG_ROOT}/include/libtconfig")

if(EXISTS "${LIBTCONFIG_CMAKE_DIR}/LIBTCONFIG-config.local")
	set(LIBTCONFIG_INCLUDE_DIRS "/home/nik/work/priklad/stereo_cam/applied_robotics-libtconfig-7df806401f29/include/libtconfig")
endif()

set(LIBTCONFIG_LIBRARIES
  "tconfig"
)

#set(LIBTCONFIG_LIBRARIES
#  "${LIBTCONFIG_CMAKE_DIR}/../../../lib/tconfig"
#  ""
#)

list(REMOVE_DUPLICATES LIBTCONFIG_INCLUDE_DIRS)
list(REMOVE_DUPLICATES LIBTCONFIG_LIBRARIES)
