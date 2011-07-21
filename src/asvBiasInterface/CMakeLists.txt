# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "biasInterface")
PROJECT(${KEYWORD})


LINK_LIBRARIES(${GtkPlus_LIBRARIES})
LINK_DIRECTORIES(${GTKPLUS_LIBRARY_DIR})
INCLUDE_DIRECTORIES(${GtkPlus_INCLUDE_DIRS})

#set up gthread
#ADD_DEFINITIONS(${GTHREAD_C_FLAGS})
#LINK_LIBRARIES(${Gthread_LIBRARIES})
#INCLUDE_DIRECTORIES(${Gthread_INCLUDE_DIRS})


#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}    		 
    ${GtkPlus_INCLUDE_DIRS}
)


# Search for source code.
FILE(GLOB folder_source *.cpp *.cc .c)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
    TARGET_LINK_LIBRARIES( ${KEYWORD}
      ${YARP_LIBRARIES}
      ${GTKPLUS_LINK_FLAGS}
      ${GtkPlus_LIBRARIES}
    )	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


INSTALL_TARGETS(/bin ${KEYWORD})

