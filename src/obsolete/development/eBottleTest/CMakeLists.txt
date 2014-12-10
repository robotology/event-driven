# Copyright: (C) 2012 RobotCub Consortium
# Authors: Rea Francesco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME eBottleTest)

PROJECT(${PROJECTNAME})


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${EMORPHLIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${PROJECTNAME}
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES(${PROJECTNAME}
      ${YARP_LIBRARIES}
                     eMorphLib
      )

    #INSTALL_TARGETS(/bin ${KEYWORD})
    install(TARGETS ${PROJECTNAME} DESTINATION bin)
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
