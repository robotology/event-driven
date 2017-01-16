#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

add_subdirectory(vPepper)

if(OPENCV_FOUND)
    add_subdirectory(vUndistortCam)
    add_subdirectory(vFramer)
endif(OPENCV_FOUND)



#if(V_USE_QTCREATOR)
#    file(GLOB xmldocumentation ${CMAKE_CURRENT_SOURCE_DIR}/*/*.xml)
#    add_custom_target(doctoken SOURCES ${xmldocumentation})
#endif(V_USE_QTCREATOR)
