# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta, Rea Francesco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "aexGrabber")
PROJECT(${KEYWORD})

message("ICUB_INCLUDE_DIRS ${ICUB_INCLUDE_DIRS}")
message("INTERNAL_INCLUDE_DIRS ${INTERNAL_INCLUDE_DIRS}")
set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${ICUB_DIR}/include
	${ICUB_CONTRIB_DIR}
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
      ${YARP_LIBRARIES} 
      eMorphLib                      
    )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
