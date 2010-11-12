cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

####################################################

set(KINECT_NODE     kinect_node)
set(KINECT_NODE_SRC src/kinect_node.cpp
                    src/kinect.cpp)

rosbuild_add_executable(${KINECT_NODE} ${KINECT_NODE_SRC})

########## EXTERNAL DEPENDENCIES ##################

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(LIBUSB      REQUIRED libusb-1.0)

include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})
target_link_libraries(${KINECT_NODE} ${LIBUSB_LIBRARIES})

target_link_libraries(${KINECT_NODE} freenect)



