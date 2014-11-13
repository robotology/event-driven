#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)


IF(NOT WIN32) 
	add_subdirectory   (  aexGrabber                   )		
#	add_subdirectory   (  asvGrabber                   )
#	add_subdirectory   (  basicAexGrabber              )
ENDIF (NOT WIN32)

#add_subdirectory  ( device2yarp )
#add_subdirectory  ( zBrdInterface )
#add_subdirectory  ( dvsGrabber )
#add_subdirectory  ( eventSniffer )

if(ICUB_USE_IPOPT)
#        add_subdirectory ( targetFinder                  )
#	add_subdirectory (  wingsTranslator              )
endif(ICUB_USE_IPOPT)

