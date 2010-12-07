cmake_minimum_required(VERSION 2.6)

project(eMorph)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

include(iCubHelpers)

# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")
				
add_subdirectory(eMorphApplication)
add_subdirectory(iKart)

## make all applications show up as single target
icub_app_all()



