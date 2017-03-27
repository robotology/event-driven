#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

if(V_BUILD_ZYNQ OR V_USE_QTCREATOR)
    add_subdirectory(zynqGrabber)
endif(V_BUILD_ZYNQ OR V_USE_QTCREATOR)

if(V_SPINNTERFACE)
    add_subdirectory(spinterface)
endif(V_SPINNTERFACE)
