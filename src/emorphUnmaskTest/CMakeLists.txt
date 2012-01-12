# Copyright: 2010 RobotCub Consortium
# Author: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME emorphUnmaskTest)
project(${PROJECTNAME})

find_package(YARP)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
#set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${ICUB_DIR}/include
    ${EMORPHLIB_INCLUDE_DIRS}
)

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} eMorphLib)
install_targets(/bin ${PROJECTNAME})

