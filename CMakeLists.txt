# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Note: 
# 03/10/13 : introduced YarpInstallationHelper
cmake_minimum_required(VERSION 2.6)

PROJECT(eMorph)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)


set(CMAKE_MODULE_PATH 
  ${YARP_MODULE_PATH} 
  ${ICUB_MODULE_PATH} 
  ${ICUBCONTRIB_MODULE_PATH}
)

include(ICUBcontribHelpers)
icubcontrib_set_default_prefix() #set CMAKE_INSTALL_PREFIX to the ICUBcontrib one; print warning if user modifies it


include(YarpInstallationHelpers)

find_package(GSL)
find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)
find_package(IPOPT)


include(iCubOptions) # removed the line, no longer necessary @Rea 12/12/13
include(iCubHelpers) # removed the line, no longer necessary @Rea 12/12/13
include(ICUBcontribOptions) #added @Rea 12/12/13

SET(NESTED_BUILD TRUE)
#SET(EMORPHLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/emorph_lib)
SET(EMORPHLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/emorph_lib/include)

ADD_SUBDIRECTORY(emorph_lib)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(app)

icubcontrib_finalize_export(emorphLib) #macro added 13/11/2013
icubcontrib_add_uninstall_target()     #macro added 13/11/2013