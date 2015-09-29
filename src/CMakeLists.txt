#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

add_subdirectory(grabbers)

if(NOT EMORPH_BUILD_ZYNQ)
    add_subdirectory(processing)
    add_subdirectory(visualization)
    add_subdirectory(movement)
endif(NOT EMORPH_BUILD_ZYNQ)

