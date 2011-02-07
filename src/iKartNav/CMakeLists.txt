cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME iKartNav)

PROJECT(${PROJECTNAME})

#SET(folder_source main.cpp)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

FIND_PACKAGE(YARP)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
