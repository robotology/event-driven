#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

add_subdirectory(eventAnalysis )
add_subdirectory(eventClustering )
add_subdirectory(template)
add_subdirectory(dPepper)
if(OPENCV_FOUND)
    add_subdirectory(vUndistortCam)
	add_subdirectory(circle)
endif(OPENCV_FOUND)

