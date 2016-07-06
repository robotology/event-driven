#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

if(OPENCV_FOUND)
    add_subdirectory( vFramer )
endif(OPENCV_FOUND)
add_subdirectory(depthGroundTruth)

