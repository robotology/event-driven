# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME spinterface)
PROJECT(${PROJECTNAME})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS}
                    /home/aglover/workspace/projects/SpinnIO/library_src/include)

add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES} /home/aglover/workspace/projects/SpinnIO/build/lib/libSpinnIO.a sqlite3 pthread)
#target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} lib-event-driven sqlite3 pthread)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
