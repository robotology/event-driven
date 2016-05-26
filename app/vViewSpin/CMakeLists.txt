# Copyright: (C) 2010 RobotCub Consortium
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(appname vViewSpin)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
add_custom_target(${appname}_dummy SOURCES ${conf} ${scripts})

icub_app_install(${appname}
				FILES ${conf}
				DESTINATION conf)
icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts/)

yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
