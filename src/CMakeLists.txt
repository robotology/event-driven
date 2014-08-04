#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)


IF(NOT WIN32) 
#	add_subdirectory   (  aexGrabber                   )		
#	add_subdirectory   (  asvGrabber                   )
#	add_subdirectory   (  basicAexGrabber              )
ENDIF (NOT WIN32)

#add_subdirectory   (  development               )
add_subdirectory   (  grabbers                  )
add_subdirectory   (  interfaces		)
add_subdirectory   (  processing                )
add_subdirectory   (  visualization             )
add_subdirectory   (  motorActions		)

