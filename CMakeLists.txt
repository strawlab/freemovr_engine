CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6)

#set(ROS_BUILD_TYPE Debug)

SET(BUILD_WITH_ROS ON CACHE BOOL "Building with ROS")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
SET(CUDA_TOOLKIT_ROOT "/usr/local/cuda")

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
FIND_PACKAGE(OsgCompute)

SET(POCO_FIND_REQUIRED 1)
FIND_PACKAGE(Poco)

SET(JANSSON_FIND_REQUIRED 1)
FIND_PACKAGE(jansson)

SET(OSG_LIBS ${OPENTHREADS_LIBRARIES} ${OSG_LIBRARIES} ${OSGVIEWER_LIBRARIES} ${OSGGA_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGWIDGET_LIBRARIES} ${OSGUTIL_LIBRARIES} ${OSGTEXT_LIBRARIES})
SET(EXR_LIBS ${OPENEXR_LIBRARIES})

SET(DS_SOURCES
  src/dsosg.cpp
  src/WindowCaptureCallback.cpp
  src/StimulusInterface.cpp
  src/InvalidBoundsCallback.cpp
  src/load_cubemap.cpp
  src/ResourceLoader.cpp
  src/ProjectCubemapToGeometryPass.cpp
  src/DisplaySurfaceGeometry.cpp
  src/util.cpp
  src/exrutil.cpp
  src/camera_model.cpp
  src/TexturedGeometryToCameraImagePass.cpp
  src/CameraImageToDisplayImagePass.cpp
  src/GeometryTextureToDisplayImagePass.cpp)

IF(OSG_COMPUTE_FOUND)
  MESSAGE("Building With CUDA support")
  add_definitions( -DFLYVR_USE_CUDA )
ELSE(OSG_COMPUTE_FOUND)
  MESSAGE("Building Without CUDA support")
ENDIF(OSG_COMPUTE_FOUND)

ADD_LIBRARY(DisplaySurfaceArbitraryGeometry SHARED src/DisplaySurfaceArbitraryGeometry.cpp)
TARGET_LINK_LIBRARIES(DisplaySurfaceArbitraryGeometry ${OSG_LIBS})

SET(CYTHON_EXECUTABLE cython) # hack. Should use CMAKE_FIND_PROGRAM, but that ignores $PATH.

IF(BUILD_WITH_ROS)
  MESSAGE("Building With ROS")

  INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  #set(ROS_BUILD_TYPE RelWithDebInfo)

  rosbuild_init()

  #set the default path for built executables to the "bin" directory
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_genmsg()
  rosbuild_gensrv()

  # Add display_server
  ADD_CUSTOM_COMMAND(OUTPUT display_server.cpp
    COMMAND ${CYTHON_EXECUTABLE} --embed --cplus -o display_server.cpp "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx")

  # the flyvr library (for building stimulus plugins) --------------------
  rosbuild_add_library(flyvr src/ResourceLoader.cpp src/StimulusInterface.cpp src/InvalidBoundsCallback.cpp src/load_cubemap.cpp)
  TARGET_LINK_LIBRARIES(flyvr ${POCO_LIBRARIES})

  # default stimulus plugins --------------------
  rosbuild_add_library(Stimulus3DDemo src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  rosbuild_add_library(Stimulus3DShaderDemo src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  rosbuild_add_library(Stimulus2DBlit src/Stimulus2DBlit.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  rosbuild_add_library(StimulusLatencyTimestamp src/StimulusLatencyTimestamp.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(StimulusLatencyTimestamp ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  rosbuild_add_library(StimulusStandby src/StimulusStandby.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  rosbuild_add_library(StimulusTemplate src/StimulusTemplate.cpp)
  TARGET_LINK_LIBRARIES(StimulusTemplate ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  # build the main display server node ----------
  rosbuild_add_executable(display_server display_server.cpp ${DS_SOURCES})

  FIND_PACKAGE(PythonLibs REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES(
    ${OSG_COMPUTE_INCLUDE_PATHS} ${OPENEXR_INCLUDE_PATHS}
  )
  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
    ${OPENEXR_LIBRARIES} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES}
    ${OSG_COMPUTE_LIBRARIES} DisplaySurfaceArbitraryGeometry)

  # add dynamic reconfigure api
  rosbuild_find_ros_package(dynamic_reconfigure)
ELSE(BUILD_WITH_ROS)
  MESSAGE("Building Without ROS")

  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

  # the flyvr library (for building stimulus plugins) --------------------
  ADD_LIBRARY(flyvr SHARED src/ResourceLoader.cpp src/StimulusInterface.cpp src/InvalidBoundsCallback.cpp src/load_cubemap.cpp)
  TARGET_LINK_LIBRARIES(flyvr ${POCO_LIBRARIES})

  # default stimulus plugins --------------------
  ADD_LIBRARY(Stimulus3DDemo SHARED src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  ADD_LIBRARY(Stimulus3DShaderDemo SHARED src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  ADD_LIBRARY(Stimulus2DBlit SHARED src/Stimulus2DBlit.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  ADD_LIBRARY(StimulusStandby SHARED src/StimulusStandby.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${POCO_LIBRARIES} flyvr)

  # build the main display server node ----------
  ADD_EXECUTABLE(display_server src/main.cpp ${DS_SOURCES})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
  INCLUDE_DIRECTORIES(
    ${OSG_COMPUTE_INCLUDE_PATHS} ${OPENEXR_INCLUDE_PATHS}
  )
  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
    ${OPENEXR_LIBRARIES} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES}
    ${OSG_COMPUTE_LIBRARIES})

ENDIF(BUILD_WITH_ROS)

# build non-ROS programs  ----------
ADD_LIBRARY(noaffinity SHARED src/noaffinity.c)

# Generate .pyx -> .cpp
ADD_CUSTOM_COMMAND(OUTPUT PyDisplaySurfaceArbitraryGeometry.cpp
  COMMAND ${CYTHON_EXECUTABLE} --cplus -o PyDisplaySurfaceArbitraryGeometry.cpp "${CMAKE_CURRENT_SOURCE_DIR}/src/PyDisplaySurfaceArbitraryGeometry.pyx"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/PyDisplaySurfaceArbitraryGeometry.pyx")

# Compile the generated .cpp
ADD_LIBRARY(PyDisplaySurfaceArbitraryGeometry SHARED PyDisplaySurfaceArbitraryGeometry.cpp)
TARGET_LINK_LIBRARIES(PyDisplaySurfaceArbitraryGeometry ${OSG_LIBS} DisplaySurfaceArbitraryGeometry)
set_target_properties(PyDisplaySurfaceArbitraryGeometry PROPERTIES PREFIX "")
