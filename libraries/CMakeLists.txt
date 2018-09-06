# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

set(VLIB_CLOCK_PERIOD_NS 80 CACHE INTEGER "event timestamp clock period (ns)")
set(VLIB_TIMER_BITS 30 CACHE INTEGER "event timestamp maximum = 2^TIMERBITS")

list(APPEND CodecTypes "CODEC_128x128" "CODEC_304x240_20" "CODEC_304x240_24")
list(GET CodecTypes 2 VLIB_CODEC_DEFAULT)
set(VLIB_CODEC_TYPE ${VLIB_CODEC_DEFAULT} CACHE STRING "select codec type")
set_property(CACHE VLIB_CODEC_TYPE PROPERTY STRINGS ${CodecTypes})

string(COMPARE GREATER ${VLIB_TIMER_BITS} 31 TOOMANYBITSINCOUNTER)

if(TOOMANYBITSINCOUNTER)
  message( FATAL_ERROR "Maximum bits useable is 31 (VLIB_TIMER_BITS)")
endif()

project(${EVENTDRIVEN_LIBRARIES})

file(GLOB folder_source
  src/vtsHelper.cpp
  src/codecs/codec_*.cpp
  src/vWindow_adv.cpp
  src/vWindow_basic.cpp
  src/vPort.cpp
  src/vCodec.cpp
  #src/vSync.cpp
)

file(GLOB folder_header
  include/iCub/eventdriven/vtsHelper.h
  include/iCub/eventdriven/vCodec.h
  include/iCub/eventdriven/vBottle.h
  include/iCub/eventdriven/vWindow_adv.h
  include/iCub/eventdriven/vWindow_basic.h
  include/iCub/eventdriven/vFilters.h
  include/iCub/eventdriven/vSurfaceHandlerTh.h
  include/iCub/eventdriven/vCollectSend.h
  include/iCub/eventdriven/vPort.h
  #include/iCub/eventdriven/vSync.h
  include/iCub/eventdriven/all.h
)

# Declare groups of source and header files -- makes things pretty in MSVC.
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Create everything needed to build our library
add_library(${EVENTDRIVEN_LIBRARIES} ${folder_source} ${folder_header})

add_definitions("-D${VLIB_CODEC_TYPE}")
add_definitions( -DCLOCK_PERIOD=${VLIB_CLOCK_PERIOD_NS} )
add_definitions( -DTIMER_BITS=${VLIB_TIMER_BITS} )

target_link_libraries(${EVENTDRIVEN_LIBRARIES} ${YARP_LIBRARIES})

if(ICUBcontrib_FOUND)
    icubcontrib_export_library(${EVENTDRIVEN_LIBRARIES}
        INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
#        EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS}
        DESTINATION include/iCub/eventdriven
        FILES  ${folder_header}
    )
    icubcontrib_finalize_export(${EVENTDRIVEN_LIBRARIES})
endif(ICUBcontrib_FOUND)
