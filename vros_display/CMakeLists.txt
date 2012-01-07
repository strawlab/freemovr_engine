cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

FIND_PACKAGE(OpenThreads)
FIND_PACKAGE(osg)
FIND_PACKAGE(osgUtil)
FIND_PACKAGE(osgDB)
FIND_PACKAGE(osgText)
FIND_PACKAGE(osgParticle)
FIND_PACKAGE(osgTerrain)
FIND_PACKAGE(osgViewer)
FIND_PACKAGE(osgGA)
FIND_PACKAGE(osgWidget)

FIND_PACKAGE(GLU)
FIND_PACKAGE(GLUT)

FIND_PACKAGE(OpenEXR)

SET(POCO_FIND_REQUIRED 1)
FIND_PACKAGE(Poco)

SET(JANSSON_FIND_REQUIRED 1)
FIND_PACKAGE(jansson)

SET(OSG_LIBS ${OPENTHREADS_LIBRARIES} ${OSG_LIBRARIES} ${OSGVIEWER_LIBRARIES} ${OSGGA_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGWIDGET_LIBRARIES} ${OSGUTIL_LIBRARIES} ${OSGTEXT_LIBRARIES})
SET(EXR_LIBS ${OPENEXR_LIBRARIES})

rosbuild_add_boost_directories()

set(CYTHON_EXECUTABLE cython) # hack. Should use CMAKE_FIND_PROGRAM, but that ignores $PATH.

# Add display_server
add_custom_command(OUTPUT display_server.cpp
  COMMAND ${CYTHON_EXECUTABLE} --embed --cplus -o display_server.cpp "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx")

SET(DS_SOURCES
  src/ProjectCubemapToGeometryPass.cpp
  src/display_screen_geometry.cpp
  src/util.cpp
  src/exrutil.cpp
  src/camera_model.cpp
  src/TexturedGeometryToCameraImagePass.cpp
  src/CameraImageToDisplayImagePass.cpp
  src/GeometryTextureToDisplayImagePass.cpp
  )

# ${OPENEXR_INCLUDE_PATHS}

# the vros_display library (for building stimulus plugins) --------------------

rosbuild_add_library(vros_display src/stimulus_interface.cpp)
target_link_libraries(vros_display)
rosbuild_link_boost(vros_display filesystem)

# default stimulus plugins --------------------

rosbuild_add_library(stimulus_3d_demo src/stimulus_3d_demo.cpp src/util.cpp)
target_link_libraries(stimulus_3d_demo ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)
rosbuild_link_boost(stimulus_3d_demo system filesystem)

rosbuild_add_library(stimulus_2d_blit src/stimulus_2d_blit.cpp src/base64.cpp)
target_link_libraries(stimulus_2d_blit ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} vros_display)

rosbuild_add_library(stimulus_standby src/stimulus_standby.cpp)
target_link_libraries(stimulus_standby ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

# build the main display server node ----------

rosbuild_add_executable(display_server display_server.cpp src/dsosg.cpp ${DS_SOURCES})
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES} ${OPENEXR_LIBRARIES} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES})
rosbuild_link_boost(display_server system filesystem)

# build other calibration programs  ----------

rosbuild_add_executable(caldc4_manual_camera_calibration src/caldc4_manual_camera_calibration.cpp
  src/display_screen_geometry.cpp
  src/util.cpp
  src/camera_model.cpp
  )
target_link_libraries(caldc4_manual_camera_calibration ${OSG_LIBS} ${JANSSON_LIBRARIES})
rosbuild_link_boost(caldc4_manual_camera_calibration program_options)


rosbuild_add_executable(caldc4_manual_projector_calibration src/caldc4_manual_projector_calibration.cpp
  )
target_link_libraries(caldc4_manual_projector_calibration ${OSG_LIBS} ${JANSSON_LIBRARIES})
rosbuild_link_boost(caldc4_manual_projector_calibration program_options system filesystem)


rosbuild_add_executable(create_p2g src/create_p2g.cpp
  src/display_screen_geometry.cpp
  src/util.cpp
  src/exrutil.cpp
  src/camera_model.cpp
  src/TexturedGeometryToCameraImagePass.cpp
  src/CameraImageToDisplayImagePass.cpp
  )
include_directories(${OPENEXR_INCLUDE_PATHS})
target_link_libraries(create_p2g ${OSG_LIBS} ${OPENEXR_LIBRARIES} ${JANSSON_LIBRARIES})

rosbuild_add_executable(calib_test_osg opengl_and_hartleyzisserman/calib_test_osg.cpp
  src/display_screen_geometry.cpp
  src/util.cpp
  src/camera_model.cpp
  )
target_link_libraries(calib_test_osg ${OSG_LIBS} ${JANSSON_LIBRARIES})

rosbuild_add_executable(calib_test_opengl opengl_and_hartleyzisserman/calib_test_opengl.c)
target_link_libraries(calib_test_opengl ${GLU_LIBRARY} ${GLUT_LIBRARIES})

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
