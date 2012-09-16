# Copyright: (C) 2012 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(appname ikartDvsObstacleDetector)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

icub_app_install(${appname}
				FILES ${conf}
				DESTINATION conf)
icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts/)
