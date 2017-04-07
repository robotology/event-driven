# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

option(VLIB_10BITCODEC "Build codec with 10bit address space" OFF)
option(VLIB_32BITTIME "Build codec using a 32bit timestamp" OFF)

project(${EVENTDRIVEN_LIBRARIES})

file(GLOB folder_source
  src/vtsHelper.cpp
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
add_library(${EVENTDRIVEN_LIBRARIES} ${folder_source} ${folder_header})

if(VLIB_32BITTIME)
    set_target_properties(${EVENTDRIVEN_LIBRARIES} PROPERTIES COMPILE_DEFINITIONS TIME32BIT)
endif(VLIB_32BITTIME)

if(VLIB_10BITCODEC)
    set_target_properties(${EVENTDRIVEN_LIBRARIES} PROPERTIES COMPILE_DEFINITIONS TENBITCODEC)
endif(VLIB_10BITCODEC)

target_link_libraries(${EVENTDRIVEN_LIBRARIES} ${YARP_LIBRARIES})

if(ICUBcontrib_FOUND)
    icubcontrib_export_library(${EVENTDRIVEN_LIBRARIES}
        INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
        EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS}
        DESTINATION include/iCub/eventdriven
        FILES  ${folder_header}
    )
    icubcontrib_finalize_export(${EVENTDRIVEN_LIBRARIES})
endif(ICUBcontrib_FOUND)
