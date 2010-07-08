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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(GROUNDSTATION groundstation)

set(GROUNDSTATION_SRC   src/gui/gtkaltimeter.c
								src/gui/gtkvariometer.c
								src/gui/gtkcompass.c
								src/gui/gtkgauge.c	
								src/gui/gtkbargauge.c	
								src/gui/gtkartificialhorizon.c	
								src/gui/gtkturncoordinator.c																							
								src/gui/AppData.cpp
								src/ground_station.cpp)

rosbuild_add_executable(${GROUNDSTATION}  ${GROUNDSTATION_SRC})

set(FAKEPOSE fakepose)
set(FAKEPOSEN_SRC   src/fake_alti.cpp)

rosbuild_add_executable(${FAKEPOSE}  ${FAKEPOSEN_SRC})

########## EXTERNAL DEPENDENCIES ##################

pkg_check_modules(GTK      REQUIRED gtk+-2.0)
include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})
target_link_libraries(${GROUNDSTATION} ${GTK_LIBRARIES})
