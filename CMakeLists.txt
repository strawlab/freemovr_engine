cmake_minimum_required(VERSION 2.8)
project(freemovr_engine)

SET(BUILD_WITH_ROS ON CACHE BOOL "Building with ROS")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/freemovr_engine/cmake/Modules/")
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
FIND_PACKAGE(osgAnimation)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLU)

FIND_PACKAGE(OpenEXR)
FIND_PACKAGE(OsgCompute)

SET(POCO_FIND_REQUIRED 1)
FIND_PACKAGE(Poco)

SET(JANSSON_FIND_REQUIRED 1)
FIND_PACKAGE(jansson)

SET(OSG_LIBS ${OPENTHREADS_LIBRARIES} ${OPENGL_LIBRARIES} ${OSG_LIBRARIES} ${OSGVIEWER_LIBRARIES} ${OSGGA_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGWIDGET_LIBRARIES} ${OSGUTIL_LIBRARIES} ${OSGTEXT_LIBRARIES} ${OSGANIMATION_LIBRARIES})

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
  add_definitions( -DFREEMOVR_USE_CUDA )
ELSE(OSG_COMPUTE_FOUND)
  MESSAGE("Building Without CUDA support")
ENDIF(OSG_COMPUTE_FOUND)

add_definitions( -DFREEMOVR_HACK_AVOID_SEGFAULT )

ADD_LIBRARY(DisplaySurfaceArbitraryGeometry SHARED src/DisplaySurfaceArbitraryGeometry.cpp)
TARGET_LINK_LIBRARIES(DisplaySurfaceArbitraryGeometry ${OSG_LIBS})

SET(CYTHON_EXECUTABLE cython) # hack. Should use CMAKE_FIND_PROGRAM, but that ignores $PATH.

IF(BUILD_WITH_ROS)
  MESSAGE("Building With ROS")
  find_package(catkin REQUIRED message_generation sensor_msgs geometry_msgs roscpp)

  add_message_files()
  add_service_files()

  catkin_python_setup()

  generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

  # see http://wiki.ros.org/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python
  catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES add_two_ints _add_two_ints_wrapper_cpp
    CATKIN_DEPENDS roscpp message_runtime geometry_msgs
    #  DEPENDS system_lib
    LIBRARIES freemovr_engine Stimulus3DDemo StimulusAxes Stimulus3DShaderDemo Stimulus2DBlit
      StimulusLatencyTimestamp StimulusStandby StimulusTemplate StimulusOSG
    )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    )

  catkin_install_python(PROGRAMS nodes/calibration_generate_exr.py
    nodes/demo_switch_calibration nodes/generate_masks.py
    # TODO add the rest of the programs in nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



  # the freemovr_engine library (for building stimulus plugins) --------------------
  add_library(freemovr_engine src/ResourceLoader.cpp src/StimulusInterface.cpp src/InvalidBoundsCallback.cpp src/load_cubemap.cpp)
  TARGET_LINK_LIBRARIES(freemovr_engine ${POCO_LIBRARIES})

  # default stimulus plugins --------------------
  add_library(Stimulus3DDemo src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  add_library(StimulusAxes src/StimulusAxes.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(StimulusAxes ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  add_library(Stimulus3DShaderDemo src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  add_library(Stimulus2DBlit src/Stimulus2DBlit.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} freemovr_engine)

  add_library(StimulusLatencyTimestamp src/StimulusLatencyTimestamp.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(StimulusLatencyTimestamp ${OSG_LIBS} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES} freemovr_engine)

  add_library(StimulusStandby src/StimulusStandby.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  add_library(StimulusTemplate src/StimulusTemplate.cpp)
  TARGET_LINK_LIBRARIES(StimulusTemplate ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  add_library(StimulusOSG src/StimulusOSG.cpp)
  TARGET_LINK_LIBRARIES(StimulusOSG ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  add_library(StimulusOSG2 src/StimulusOSG2.cpp)
  TARGET_LINK_LIBRARIES(StimulusOSG2 ${OSG_LIBS} ${POCO_LIBRARIES} freemovr_engine)

  # build the main display server node ----------
  ADD_EXECUTABLE(display_server display_server.cpp ${DS_SOURCES})

  find_package(PythonLibs 2.7 REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES(
    ${OSG_COMPUTE_INCLUDE_PATHS} ${OPENEXR_INCLUDE_PATHS}
  )
  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
      ${OPENEXR_LIBRARIES} ${POCO_LIBRARIES} ${JANSSON_LIBRARIES}
      ${OSG_COMPUTE_LIBRARIES} DisplaySurfaceArbitraryGeometry)

  # build another small executable for parsing osg files for animations
  add_executable(parseosg src/parseosg.cpp)
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
  TARGET_LINK_LIBRARIES(parseosg ${OSG_LIBS})

  install(TARGETS display_server freemovr_engine Stimulus3DDemo StimulusAxes Stimulus3DShaderDemo
    Stimulus2DBlit StimulusLatencyTimestamp StimulusStandby StimulusTemplate StimulusOSG
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


  install(TARGETS PyDisplaySurfaceArbitraryGeometry DisplaySurfaceArbitraryGeometry
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(DIRECTORY data
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  install(DIRECTORY src/shaders
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)

ELSE(BUILD_WITH_ROS)
  MESSAGE("Building Without ROS")

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

# Add display_server
ADD_CUSTOM_COMMAND(OUTPUT display_server.cpp
  COMMAND ${CYTHON_EXECUTABLE} --embed --cplus -o display_server.cpp "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx")


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
