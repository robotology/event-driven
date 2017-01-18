# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME eMorph)

include(YarpInstallationHelpers)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.m)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

if(EMORPH_USE_QTCREATOR)
    add_custom_target(apptoken SOURCES ${scripts} ${conf})
endif(EMORPH_USE_QTCREATOR)

