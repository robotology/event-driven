# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME eMorphLib)
PROJECT(${PROJECTNAME})

SET(PROJECT_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source 
  src/eventBuffer.cpp
  src/eventConversion.cpp
  src/eventCodec.cpp
  src/eventBottle.cpp
)
FILE(GLOB folder_header 
  include/*.h
)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

message ("YARP_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ")
# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
	${YARP_INCLUDE_DIRS}
        ${ICUB_INCLUDE_DIRS}
        ${ICUB_CONTRIB}/include
)

# Create everything needed to build our executable.
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

# Makes VS a bit quiter wrt warnings.
IF(MSVC)
    SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})

MESSAGE(STATUS "icub_dir: ${ICUB_DIR}")
include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
                    
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
                                   DESTINATION include/iCub
                                   FILES ${folder_header})
