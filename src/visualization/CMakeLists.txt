#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)


IF(NOT WIN32) 
#	add_subdirectory   (  aexGrabber                   )		
#	add_subdirectory   (  asvGrabber                   )
#	add_subdirectory   (  basicAexGrabber              )
ENDIF (NOT WIN32)


add_subdirectory  (  basicCartesianCollector      )
add_subdirectory  (  cartesian_frame_converter    )
add_subdirectory  (  cartesian_frame_printer      )
add_subdirectory  (  cartesianFrameCollector      )
add_subdirectory  (  logpolar_frame_converter     )
add_subdirectory  (  logpolar_frame_converter_add )
add_subdirectory  (  logpolarFrameCollector	  )
add_subdirectory  (  logSort			  )

