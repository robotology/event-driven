#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

#add_subdirectory(aexGrabber)
add_subdirectory(zynqGrabber)

if(EMORPH_SPINNTERFACE)
    add_subdirectory(spinterface)
endif(EMORPH_SPINNTERFACE)
