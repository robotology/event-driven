# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

option(VLIB_10BITCODEC "Build codec with 10bit address space" OFF)
option(VLIB_32BITTIME "Build codec using a 32bit timestamp" OFF)

set(PROJECTNAME ${EVENTDRIVEN_LIBRARIES})
project(${PROJECTNAME})

file(GLOB folder_source
  src/codecs/codec_*.cpp
  src/vWindow_adv.cpp
  src/vWindow_basic.cpp
)

file(GLOB folder_header
  include/iCub/eventdriven/vtsHelper.h
  include/iCub/eventdriven/vCodec.h
  include/iCub/eventdriven/vBottle.h
  include/iCub/eventdriven/vWindow_adv.h
  include/iCub/eventdriven/vWindow_basic.h
  include/iCub/eventdriven/vFilters.h
  include/iCub/eventdriven/vSurfaceHandlerTh.h
  include/iCub/eventdriven/all.h
)

# Declare groups of source and header files -- makes things pretty in MSVC.
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
)

# Create everything needed to build our library
add_library(${PROJECTNAME} ${folder_source} ${folder_header})

# Makes VS a bit quiter wrt warnings.
if(MSVC)
    set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
endif(MSVC)

if(VLIB_10BITCODEC)
    set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_DEFINITIONS TENBITCODEC)
endif(VLIB_10BITCODEC)

if(VLIB_32BITTIME)
    set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_DEFINITIONS TIME32BIT)
endif(VLIB_32BITTIME)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

if(ICUBcontrib_FOUND)
    icubcontrib_export_library(${PROJECTNAME}
        INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
        EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS}
        DESTINATION include/iCub/eventdriven
        FILES  ${folder_header}
    )
    icubcontrib_finalize_export(${PROJECTNAME})
endif(ICUBcontrib_FOUND)
