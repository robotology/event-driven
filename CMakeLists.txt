# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(CONTEXT_DIR eventdriven)
set(PROJECTNAME icub-event-driven)
project(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fno-strict-aliasing -std=c++11 -fPIC")
set(EVENTDRIVENLIBS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/include)
set(EVENTDRIVEN_LIBRARIES eventdriven)

option(USE_QTCREATOR "Add apps/drivers to QtCreator IDE" OFF)

#YARP
find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)

#ICUBCONTRIB
find_package(ICUBcontrib)
if(ICUBcontrib_FOUND)
    list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
    include(ICUBcontribOptions)
    include(ICUBcontribHelpers)
    icubcontrib_set_default_prefix()
else(ICUBcontrib_FOUND)
    message("Warning: ICUBcontrib not found. Make Install may not install to the correct location")
endif(ICUBcontrib_FOUND)

add_subdirectory(libraries)
add_subdirectory(src)

icubcontrib_add_uninstall_target()     #macro added 13/11/2013
