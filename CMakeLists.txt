CMAKE_MINIMUM_REQUIRED(VERSION 2.8.10)
PROJECT(flyvr)
SET(BUILD_WITH_ROS OFF CACHE BOOL "Building with ROS")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
SET(CUDA_TOOLKIT_ROOT "/usr/local/cuda")

add_definitions(-DUNICODE -D_UNICODE) # use unicode in MS projects

include (GenerateExportHeader)

FIND_PACKAGE(OpenThreads)
# FIND_PACKAGE(osg)
# FIND_PACKAGE(osgUtil)
# FIND_PACKAGE(osgDB)
# FIND_PACKAGE(osgText)
# FIND_PACKAGE(osgParticle)
# FIND_PACKAGE(osgTerrain)
# FIND_PACKAGE(osgViewer)
# FIND_PACKAGE(osgGA)
# FIND_PACKAGE(osgWidget)
find_package(OpenSceneGraph 3.0.1 REQUIRED osg osgDB osgUtil osgParticle osgTerrain osgViewer osgGA osgWidget)

FIND_PACKAGE(OPENGL)
FIND_PACKAGE(GLUT)

#IF(!WIN32)
	SET(OPENEXR_FIND_REQUIRED 1)
	FIND_PACKAGE(OpenEXR)
#ENDIF(!WIN32)

#FIND_PACKAGE(OsgCompute)

#SET(POCO_FIND_REQUIRED Net)
#FIND_PACKAGE(Poco)
find_package(Poco REQUIRED Net) 

#SET(JANSSON_FIND_REQUIRED 1)
FIND_PACKAGE(jansson)

SET(OSG_LIBS ${OPENTHREADS_LIBRARIES} ${OSG_LIBRARIES} ${OSGVIEWER_LIBRARIES} ${OSGGA_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGWIDGET_LIBRARIES} ${OSGUTIL_LIBRARIES} ${OSGTEXT_LIBRARIES})
SET(EXR_LIBS ${OPENEXR_LIBRARIES})

SET(DS_SOURCES
  src/dsosg.cpp
  src/WindowCaptureCallback.cpp
  src/StimulusInterface.cpp
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
  add_definitions( -DFLYVR_USE_CUDA )
ENDIF(OSG_COMPUTE_FOUND)

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

  SET(CYTHON_EXECUTABLE cython) # hack. Should use CMAKE_FIND_PROGRAM, but that ignores $PATH.

  # Add display_server
  ADD_CUSTOM_COMMAND(OUTPUT display_server.cpp
    COMMAND ${CYTHON_EXECUTABLE} --embed --cplus -o display_server.cpp "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/display_server.pyx")

  # the flyvr library (for building stimulus plugins) --------------------
  rosbuild_add_library(flyvr src/ResourceLoader.cpp src/StimulusInterface.cpp src/load_cubemap.cpp)
  TARGET_LINK_LIBRARIES(flyvr ${Poco_LIBRARIES})

  # default stimulus plugins --------------------
  rosbuild_add_library(Stimulus3DDemo src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${Poco_LIBRARIES} flyvr)

  rosbuild_add_library(Stimulus3DShaderDemo src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${Poco_LIBRARIES} flyvr)

  rosbuild_add_library(Stimulus2DBlit src/Stimulus2DBlit.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${Poco_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  rosbuild_add_library(StimulusStandby src/StimulusStandby.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${Poco_LIBRARIES} flyvr)

  rosbuild_add_library(StimulusTemplate src/StimulusTemplate.cpp)
  TARGET_LINK_LIBRARIES(StimulusTemplate ${OSG_LIBS} ${Poco_LIBRARIES} flyvr)

  # build the main display server node ----------
  rosbuild_add_executable(display_server display_server.cpp ${DS_SOURCES})

  FIND_PACKAGE(PythonLibs REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES(
    ${OSG_COMPUTE_INCLUDE_PATHS} ${OPENEXR_INCLUDE_PATHS}
  )
  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
    ${OPENEXR_LIBRARIES} ${Poco_LIBRARIES} ${JANSSON_LIBRARIES}
    ${OSG_COMPUTE_LIBRARIES})

  # add dynamic reconfigure api
  rosbuild_find_ros_package(dynamic_reconfigure)
ELSE(BUILD_WITH_ROS)
  MESSAGE("Building Without ROS")

  IF(WIN32)
#    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
	
	FOREACH(config ${CMAKE_CONFIGURATION_TYPES})
		STRING(TOUPPER "${config}" config_toupper)
		SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_toupper} ${PROJECT_BINARY_DIR}/${config}/lib/)
	ENDFOREACH(config)

  ELSE(WIN32)
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
	SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
  ENDIF(WIN32)
  
  # the flyvr library (for building stimulus plugins) --------------------
  #add_compiler_export_flags()
  ADD_LIBRARY(flyvr STATIC src/ResourceLoader.cpp src/StimulusInterface.cpp src/load_cubemap.cpp)
  #generate_export_header(flyvr)

  TARGET_LINK_LIBRARIES(flyvr ${OSG_LIBS} ${Poco_LIBRARIES})
	
  # default stimulus plugins --------------------
  SET(CMAKE_DEBUG_POSTFIX "d")  # all debug libraries should end with "d"
  SET(CMAKE_SHARED_LIBRARY_PREFIX "lib") # use "lib" prefix for .dll even on windows
  
  ADD_LIBRARY(Stimulus3DDemo SHARED src/Stimulus3DDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DDemo ${OSG_LIBS} ${Poco_LIBRARIES} flyvr)

  ADD_LIBRARY(Stimulus3DShaderDemo SHARED src/Stimulus3DShaderDemo.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(Stimulus3DShaderDemo ${OSG_LIBS} ${Poco_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  ADD_LIBRARY(Stimulus2DBlit SHARED src/Stimulus2DBlit.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(Stimulus2DBlit ${OSG_LIBS} ${Poco_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  ADD_LIBRARY(StimulusStandby SHARED src/StimulusStandby.cpp src/util.cpp)
  TARGET_LINK_LIBRARIES(StimulusStandby ${OSG_LIBS} ${Poco_LIBRARIES} flyvr)

  ADD_LIBRARY(StimulusOSGFile SHARED src/StimulusOSGFile.cpp src/json2osg.cpp  src/base64.cpp)
  TARGET_LINK_LIBRARIES(StimulusOSGFile ${OSG_LIBS} ${Poco_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  ADD_LIBRARY(StimulusCylinder SHARED src/StimulusCylinder.cpp src/json2osg.cpp src/base64.cpp)
  TARGET_LINK_LIBRARIES(StimulusCylinder ${OSG_LIBS} ${Poco_LIBRARIES} ${JANSSON_LIBRARIES} flyvr)

  GET_TARGET_PROPERTY(Lib_Location Stimulus3DDemo LOCATION)
  
  # build the main display server node ----------
  IF(WIN32)
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${config})
	FOREACH(config ${CMAKE_CONFIGURATION_TYPES})
		STRING(TOUPPER "${config}" config_toupper)
		SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_toupper} ${PROJECT_BINARY_DIR}/${config})
	ENDFOREACH(config)
  ENDIF(WIN32)
  ADD_EXECUTABLE(display_server src/main.cpp ${DS_SOURCES})
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")
  INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")

  IF(WIN32)
	FOREACH(config ${CMAKE_CONFIGURATION_TYPES})
		FILE(COPY data DESTINATION ${PROJECT_BINARY_DIR}/${config}/)
		FILE(COPY src/shaders DESTINATION ${PROJECT_BINARY_DIR}/${config}/src/)
		FILE(COPY config DESTINATION ${PROJECT_BINARY_DIR}/${config}/    )
	ENDFOREACH(config)
  ELSE(WIN32)
    FILE(COPY data DESTINATION ${PROJECT_BINARY_DIR})
    FILE(COPY src/shaders DESTINATION ${PROJECT_BINARY_DIR}/src/)
    FILE(COPY config DESTINATION ${PROJECT_BINARY_DIR}    )
  ENDIF(WIN32)

  # INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS} ${Poco_INCLUDE_DIRS} ${JANSSON_INCLUDE_PATHS} ${OPENEXR_INCLUDE_PATHS})
# do NOT use all OSG dirs, only the toplevel (otherwise windows C2504 errors)
 INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${Poco_INCLUDE_DIRS} ${JANSSON_INCLUDE_PATHS} ${OPENEXR_INCLUDE_PATHS})

  TARGET_LINK_LIBRARIES(display_server ${OSG_LIBS} ${PYTHON_LIBRARIES}
    ${OPENEXR_LIBRARIES} ${Poco_LIBRARIES} ${Poco_Net_LIBRARY} ${JANSSON_LIBRARIES}
    ${OSG_COMPUTE_LIBRARIES}  ${OPENGL_gl_LIBRARY})

#  MESSAGE( STATUS "OSG_COMPUTE_INCLUDE_PATHS: " ${OSG_COMPUTE_INCLUDE_PATHS} )
  MESSAGE( STATUS "OPENSCENEGRAPH_INCLUDE_DIRS: " ${OPENSCENEGRAPH_INCLUDE_DIRS} )
  MESSAGE( STATUS "OPENSCENEGRAPH_LIBRARIES: " ${OPENSCENEGRAPH_LIBRARIES} )
  MESSAGE( STATUS "Poco_INCLUDE_DIRS: " ${Poco_INCLUDE_DIRS} )
  MESSAGE( STATUS "OPENEXR_INCLUDE_PATHS: " ${OPENEXR_INCLUDE_PATHS} )
  MESSAGE( STATUS "OPENGL_gl_LIBRARY: " ${OPENGL_gl_LIBRARY} )

ENDIF(BUILD_WITH_ROS)

IF(NOT WIN32)
# build non-ROS programs  ----------
ADD_LIBRARY(noaffinity SHARED src/noaffinity.c)
ENDIF(NOT WIN32)

 # debug print all cmake variables
 
get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
    # message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

