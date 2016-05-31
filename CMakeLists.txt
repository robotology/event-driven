# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME eMorph)
project(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "-Wall -fno-strict-aliasing")
set(EMORPHLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/emorph_lib/include)

option(EMORPH_BUILD_ZYNQ "Build for ZynqGrabber" OFF)
option(EMORPH_USE_QTCREATOR "Add apps/drivers to QtCreator IDE" OFF)
option(EMORPH_SPINNTERFACE "Build the SpiNNaker Ethernet Interface" OFF)
option(EMORPH_10BITCODEC "Build codec with 10bit address space" OFF)

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

find_package(ICUBcontrib)
if(ICUBcontrib_FOUND)
    #message("iCub Module Path: "  ${ICUBCONTRIB_MODULE_PATH})
    list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
    include(ICUBcontribOptions)
    include(ICUBcontribHelpers)
    icubcontrib_set_default_prefix()
else(ICUBcontrib_FOUND)
    message("Warning: ICUBcontrib not found. Make Install may not install to the correct location")
endif(ICUBcontrib_FOUND)

if(NOT EMORPH_BUILD_ZYNQ)

    find_package(ICUB)
    if(NOT ICUB_FOUND)
        message("Warning: ICUB not found. Certain Modules will not be built")
    endif(NOT ICUB_FOUND)

    find_package(OpenCV)
    if(NOT OPENCV_FOUND)
         message("Warning: OpenCV Not Found. Certain Modules will not be built")
    endif(NOT OPENCV_FOUND)

    add_subdirectory(drivers)

endif(NOT EMORPH_BUILD_ZYNQ)

add_subdirectory(app)
add_subdirectory(emorph_lib)
add_subdirectory(src)



#icubcontrib_finalize_export(emorphLib) #macro added 13/11/2013
icubcontrib_add_uninstall_target()     #macro added 13/11/2013
