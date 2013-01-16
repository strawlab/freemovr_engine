CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6)
SET(BUILD_VROS ON CACHE BOOL "Build Using VROS")

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
  src/stimulus_interface.cpp
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

set(CUBMAP_TO_GEOMETRY_PART_SOURCES
  src/project_cubemap_onto_geometry.cpp
  src/WindowCaptureCallback.cpp
  src/load_cubemap.cpp
  src/util.cpp
  src/ResourceLoader.cpp
  src/ProjectCubemapToGeometryPass.cpp
  src/DisplaySurfaceGeometry.cpp
)

SET(P2G_SOURCES
  src/create_p2g.cpp
  src/DisplaySurfaceGeometry.cpp
  src/util.cpp
  src/exrutil.cpp
  src/camera_model.cpp
  src/TexturedGeometryToCameraImagePass.cpp
  src/CameraImageToDisplayImagePass.cpp)

SET(CALIB_TEST_SOURCES
  opengl_and_hartleyzisserman/calib_test_osg.cpp
  src/DisplaySurfaceGeometry.cpp
  src/util.cpp
  src/camera_model.cpp)

IF(BUILD_VROS)
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

  SET(CYTHON_EXECUTABLE cython) # hack. Should use CMAKE_FIND_PROGRAM, but that ignores $PATH.

  # Add display_server
  ADD_CUSTOM_COMMAND(OUTPUT display_server.cpp
    COMMAND ${CYTHON_EXECUTABLE} --embed --cplus -o display_server.cpp "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx")

  # the vros_display library (for building stimulus plugins) --------------------
  rosbuild_add_library(vros_display src/ResourceLoader.cpp src/stimulus_interface.cpp src/load_cubemap.cpp)
  TARGET_LINK_LIBRARIES(vros_display ${POCO_LIBRARIES})

  # default stimulus plugins --------------------
  rosbuild_add_library(Stimulus3DDemo src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  rosbuild_add_library(Stimulus3DShaderDemo src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  rosbuild_add_library(Stimulus2DBlit src/Stimulus2DBlit.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} vros_display)

  rosbuild_add_library(StimulusStandby src/StimulusStandby.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  rosbuild_add_library(Stimulus2DSprite src/Stimulus2DSprite.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DSprite ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  rosbuild_add_library(StimulusTemplate src/StimulusTemplate.cpp)
  TARGET_LINK_LIBRARIES(StimulusTemplate ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  # build the main display server node ----------
  rosbuild_add_executable(display_server display_server.cpp ${DS_SOURCES})

  FIND_PACKAGE(PythonLibs REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES(
    ${OSG_COMPUTE_INCLUDE_PATHS}
  )
  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
    ${OPENEXR_LIBRARIES} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES}
    ${OSG_COMPUTE_LIBRARIES})
  add_definitions( -DVROS_USE_CUDA )

  # add dynamic reconfigure api
  rosbuild_find_ros_package(dynamic_reconfigure)
ELSE(BUILD_VROS)
  MESSAGE("Building Without ROS")

  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

  # the vros_display library (for building stimulus plugins) --------------------
  ADD_LIBRARY(vros_display SHARED src/ResourceLoader.cpp src/stimulus_interface.cpp src/load_cubemap.cpp)
  TARGET_LINK_LIBRARIES(vros_display ${POCO_LIBRARIES})

  # default stimulus plugins --------------------
  ADD_LIBRARY(Stimulus3DDemo SHARED src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  ADD_LIBRARY(Stimulus3DShaderDemo SHARED src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  ADD_LIBRARY(Stimulus2DBlit SHARED src/Stimulus2DBlit.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} vros_display)

  ADD_LIBRARY(StimulusStandby SHARED src/StimulusStandby.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  ADD_LIBRARY(Stimulus2DSprite SHARED src/Stimulus2DSprite.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DSprite ${OSG_LIBS} ${POCO_LIBRARIES} vros_display)

  # build the main display server node ----------
  ADD_EXECUTABLE(display_server src/main.cpp ${DS_SOURCES})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
  INCLUDE_DIRECTORIES(
    ${OSG_COMPUTE_INCLUDE_PATHS}
  )
  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
    ${OPENEXR_LIBRARIES} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES}
    ${OSG_COMPUTE_LIBRARIES})
  add_definitions( -DVROS_USE_CUDA )

ENDIF(BUILD_VROS)

# build non-ROS programs  ----------
ADD_EXECUTABLE(project_cubemap_onto_geometry ${CUBMAP_TO_GEOMETRY_PART_SOURCES})
TARGET_LINK_LIBRARIES(project_cubemap_onto_geometry ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES})

ADD_LIBRARY(noaffinity SHARED src/noaffinity.c)

# build other calibration programs  ----------
ADD_EXECUTABLE(create_p2g ${P2G_SOURCES})
INCLUDE_DIRECTORIES(${OPENEXR_INCLUDE_PATHS})
TARGET_LINK_LIBRARIES(create_p2g ${OSG_LIBS} ${OPENEXR_LIBRARIES} ${JANSSON_LIBRARIES})

ADD_EXECUTABLE(calib_test_osg ${CALIB_TEST_SOURCES})
TARGET_LINK_LIBRARIES(calib_test_osg ${OSG_LIBS} ${JANSSON_LIBRARIES})

ADD_EXECUTABLE(calib_test_opengl opengl_and_hartleyzisserman/calib_test_opengl.c)
TARGET_LINK_LIBRARIES(calib_test_opengl ${GLU_LIBRARY} ${GLUT_LIBRARIES})
