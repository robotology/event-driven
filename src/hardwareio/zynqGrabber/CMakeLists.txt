# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME "zynqGrabber")
PROJECT(${PROJECT_NAME})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EVENTDRIVENLIBS_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
)

# Search for source code.
file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})


add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

if(V_10BITCODEC)
    set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS TENBITCODEC)
endif(V_10BITCODEC)

#the zynqGrabber should only be installed for the pc104
if(V_BUILD_ZYNQ)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(V_BUILD_ZYNQ)

