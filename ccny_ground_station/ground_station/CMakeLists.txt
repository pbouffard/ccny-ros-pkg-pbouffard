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

set(TELEMETRY telemetry_demo)
set(TELEMETRY_SRC src/gui/gtkaltimeter.c
						src/gui/gtkvariometer.c
						src/gui/gtkcompass.c
						src/gui/gtkgauge.c	
						src/gui/gtkbargauge.c	
						src/gui/gtkartificialhorizon.c	
						src/gui/gtkturncoordinator.c																							
						src/gui/telemetry_widgets_appdata.cpp
						src/telemetry_widgets.cpp)
rosbuild_add_executable(${TELEMETRY}  ${TELEMETRY_SRC})

set(GROUNDSTATION groundstation)
set(GROUNDSTATION_SRC   src/gui/gtkaltimeter.c
								src/gui/gtkvariometer.c
								src/gui/gtkcompass.c
								src/gui/gtkgauge.c	
								src/gui/gtkbargauge.c	
								src/gui/gtkartificialhorizon.c	
								src/gui/gtkturncoordinator.c																							
								src/gui/gpsd_viewer_osd.c																							
								src/gui/ground_station_appdata.cpp		
								src/gui/ground_station_callbacks.cpp						
								src/ground_station.cpp)
rosbuild_add_executable(${GROUNDSTATION}  ${GROUNDSTATION_SRC})


set(FAKEPOSE fakepose)
set(FAKEPOSEN_SRC   src/fake_alti.cpp)
rosbuild_add_executable(${FAKEPOSE}  ${FAKEPOSEN_SRC})

########## EXTERNAL DEPENDENCIES ##################

pkg_check_modules(GTK      REQUIRED gtk+-2.0)
pkg_check_modules(GMODULE  REQUIRED gmodule-2.0)
pkg_check_modules(LIBGLADE REQUIRED libglade-2.0)

include_directories(${GTK_INCLUDE_DIRS})
include_directories(${GMODULE_INCLUDE_DIRS})
include_directories(${LIBGLADE_INCLUDE_DIRS})

link_directories(${GTK_LIBRARY_DIRS})
link_directories(${GMODULE_LIBRARY_DIRS})
link_directories(${LIBGLADE_LIBRARY_DIRS})

target_link_libraries(${GROUNDSTATION} ${GTK_LIBRARIES})
target_link_libraries(${GROUNDSTATION} ${GMODULE_LIBRARIES})
target_link_libraries(${GROUNDSTATION} ${LIBGLADE_LIBRARIES})
target_link_libraries(${GROUNDSTATION} libosmgpsmap.so)

target_link_libraries(${TELEMETRY} ${GTK_LIBRARIES})

