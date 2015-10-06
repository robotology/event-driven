# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME "zynqGrabber")
PROJECT(${PROJECT_NAME})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EMORPHLIB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
)

# Search for source code.
file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

#the zynqGrabber should only be installed for the pc104
if(EMORPH_BUILD_ZYNQ)
    add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} eMorphLib)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(EMORPH_BUILD_ZYNQ)

#however we want to see it in QT CREATOR
if(EMORPH_USE_QTCREATOR AND NOT EMORPH_BUILD_ZYNQ)
    add_custom_target(${PROJECT_NAME} SOURCES ${folder_source} ${folder_header})
endif(EMORPH_USE_QTCREATOR AND NOT EMORPH_BUILD_ZYNQ)
