# Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Author:Shashank Pathak
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "dvsCalibrator")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 2.6)

FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 
    ${ICUB_INCLUDE_DIRS}    	
    ${ICUB_CONTRIB_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    #${OPENCV_INCLUDE_DIR}
    ${EMORPHLIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD}        
      eMorphLib
      ${YARP_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${GSL_LIBRARIES}
      #${OpenCV_LIBS}
      )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
