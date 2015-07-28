#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)


IF(NOT WIN32) 

if(NOT  EMORPH_USE_ZINC)
        add_subdirectory   (  vGrabber                   )
endif()
	add_subdirectory   (  zynqGrabber                )
ENDIF (NOT WIN32)

