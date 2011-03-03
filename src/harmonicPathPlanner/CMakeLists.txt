# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME "harmonicPathPlanner")
PROJECT(${PROJECTNAME})


INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS} )

# Search for source code.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${OPENCV_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

