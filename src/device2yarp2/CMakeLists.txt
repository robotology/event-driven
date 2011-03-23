# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME device2yarp)
PROJECT(${PROJECTNAME})

# first find the iCub package
find_package(ICUB)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}
    ${YARP_INCLUDE_DIRS}
)

#search for source code
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF (folder_source)
	ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
	TARGET_LINK_LIBRARIES (${PROJECTNAME} ${YARP_LIBRARIES})
	INSTALL_TARGETS(/bin ${PROJECTNAME})
ELSE (folder_souce)
	MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF(folder_source)
