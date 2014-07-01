# Copyright: (C) 2011 RobotCub Consortium
# Authors: Fouzhan Hosseini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(PROJECTNAME "eventOpticalFlow")
project(${PROJECTNAME})

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

FIND_PACKAGE(YARP)

#FIND_LIBRARY(EMORPH_LIB NAMES eMorphLib PATHS $ENV{ICUB_ROOT}/contrib/src/eMorph/emorph_lib)
#set(EMORPHLIB_INCLUDE_DIRS $ENV{ICUB_ROOT}/contrib/src/eMorph/emorph_lib/include)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS}
		    ${ICUB_INCLUDE_DIRS}
                    ${EMORPHLIB_INCLUDE_DIRS})

FILE(GLOB sourse_folder *.cpp)
FILE(GLOB header_folder *.h)

add_executable(${PROJECTNAME} ${sourse_folder} ${header_folder})
TARGET_LINK_LIBRARIES(${PROJECTNAME} eMorphLib ${YARP_LIBRARIES})

#INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)

INSTALL_TARGETS(/bin ${PROJECT_NAME})

