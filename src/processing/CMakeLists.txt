#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

add_subdirectory(vtsOptFlow)
add_subdirectory(eventAnalysis )
add_subdirectory(eventClustering )
add_subdirectory(template)
add_subdirectory(dPepper)
add_subdirectory(circle)
if(OPENCV_FOUND)
    add_subdirectory(vUndistortCam)
endif(OPENCV_FOUND)

