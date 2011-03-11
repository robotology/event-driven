# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

PROJECT(eMorph)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.
FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)
find_package(IPOPT)

include(iCubOptions)
include(iCubHelpers)

SET(NESTED_BUILD TRUE)
SET(EMORPHLIB_DIR "${ICUB_ROOT}/contrib/src/eMorph/emorph_lib")

#ADD_SUBDIRECTORY(emorph_lib)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(app)
