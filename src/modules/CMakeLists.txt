#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

add_subdirectory(vFlow)
#add_subdirectory(vAnalysis)
add_subdirectory(vCluster)
add_subdirectory(vPepper)

if(ICUB_FOUND)
    add_subdirectory(vCircle)
    add_subdirectory(vCircleDisparity)
  add_subdirectory(vParticleFilter)
endif(ICUB_FOUND)

add_subdirectory(vCorner)
add_subdirectory(vRepTest)

if(OPENCV_FOUND)
    add_subdirectory(vUndistortCam)
    add_subdirectory(vFramer)
endif(OPENCV_FOUND)
