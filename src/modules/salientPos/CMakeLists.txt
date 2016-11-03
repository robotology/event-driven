# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME salientPos)
PROJECT(${PROJECTNAME})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${EMORPHLIB_INCLUDE_DIRS}
                    /usr/local/include)

add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} eMorphLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
