cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)

####################################################

set(MVOG_GTK_LIB MVOGGTK)

set(MVOG_GTK_SRC src/gtk_gui.cpp
                 src/callbacks.cpp  
                 src/draw_callbacks.cpp 
                 src/map_drawer_3d.cpp
                 src/map_drawer_2d.cpp
                 src/camera.cpp)

rosbuild_add_library(${MVOG_GTK_LIB} ${MVOG_GTK_SRC})

########## EXTERNAL DEPENDENCIES ##################

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

pkg_check_modules(GTK      REQUIRED gtk+-2.0)
pkg_check_modules(GMODULE  REQUIRED gmodule-2.0)
pkg_check_modules(GTKGLEXT REQUIRED gtkglext-1.0)
pkg_check_modules(LIBGLADE REQUIRED libglade-2.0)

include_directories(${GTK_INCLUDE_DIRS})
include_directories(${GMODULE_INCLUDE_DIRS})
include_directories(${GTKGLEXT_INCLUDE_DIRS})
include_directories(${LIBGLADE_INCLUDE_DIRS})

link_directories(${GTK_LIBRARY_DIRS})
link_directories(${GMODULE_LIBRARY_DIRS})
link_directories(${GTKGLEXT_LIBRARY_DIRS})
link_directories(${LIBGLADE_LIBRARY_DIRS})


