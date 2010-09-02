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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

rosbuild_find_ros_package(csm_install)

SET(ENV{PKG_CONFIG_PATH} "${csm_install_PACKAGE_PATH}/lib/pkgconfig")
MESSAGE($ENV{PKG_CONFIG_PATH})

# Require we have pkgconfig installed
find_package(PkgConfig REQUIRED)

# Tell pkgconfig to look for CSM
pkg_check_modules(CSM REQUIRED csm)

IF(${CSM_FOUND})
  MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
  MESSAGE("CSM_LIBRARIES:    ${CSM_LIBRARIES}")
  MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

  include_directories(${CSM_INCLUDE_DIRS})
  link_directories(${CSM_LIBRARY_DIRS})   
ELSE(${CSM_FOUND})
  MESSAGE(FATAL_ERROR "CSM not found")
ENDIF(${CSM_FOUND})

set(CSM_NODE      csm_node)
set(CSM_PROJ_NODE csm_proj_node)

rosbuild_add_executable(${CSM_NODE}      src/csm_node.cpp)
rosbuild_add_executable(${CSM_PROJ_NODE} src/csm_node.cpp)

SET_TARGET_PROPERTIES(${CSM_PROJ_NODE} PROPERTIES
  COMPILE_DEFINITIONS "USE_PROJECTED_SCANS")

target_link_libraries(${CSM_NODE}      ${CSM_LIBRARIES})
target_link_libraries(${CSM_PROJ_NODE} ${CSM_LIBRARIES})



