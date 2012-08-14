# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME eventManager)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
   
find_package(YARP)
find_package(ICUB)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

find_package(OpenCV)
find_package(GSL)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS} 
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

set(folder_source main.cpp)
# Search for source code.
file( GLOB folder_source src/*.cpp )
file( GLOB folder_header include/iCub/*.h )

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# add executables and link libraries.
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${GSL_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION $ENV{ICUB_DIR}/bin)

