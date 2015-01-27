#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)


IF(NOT WIN32) 
        add_subdirectory   (  vGrabber                   )
ENDIF (NOT WIN32)


if(ICUB_USE_IPOPT)
#        add_subdirectory ( targetFinder                  )
#	add_subdirectory (  wingsTranslator              )
endif(ICUB_USE_IPOPT)

