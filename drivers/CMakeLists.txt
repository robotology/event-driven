# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

if(EMORPH_USE_QTCREATOR)
    file(GLOB allfiles ${CMAKE_CURRENT_SOURCE_DIR}/*/*.c ${CMAKE_CURRENT_SOURCE_DIR}/*/*.h)
    add_custom_target(driverstoken SOURCES ${allfiles})
endif(EMORPH_USE_QTCREATOR)


