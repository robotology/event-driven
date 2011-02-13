cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME iKartBatteryManager)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} icubmod
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
