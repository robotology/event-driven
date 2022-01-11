# Copyright: (C) 2019 EDPR - Istituto Italiano di Tecnologia
# Authors: Arren Glover, Daniele Domenichelli
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 3.5)
project(event-driven
        LANGUAGES CXX C
        VERSION 2.0)
include(GNUInstallDirs)

#options
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
set(CONTEXT_DIR event-driven)
set(EVENTDRIVEN_LIBRARY event-driven)

#YCM and cmake search path options
message(STATUS "Install Directory: ${CMAKE_INSTALL_PREFIX}")
find_package(YCM QUIET)
if(YCM_FOUND)
  message(STATUS "Found YCM: (found version ${YCM_VERSION})")
else()
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif() 
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set(YARP_COMPONENTS os sig math dev)
set(OpenCV_dep "")

# opencv
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV: (found version ${OpenCV_VERSION})")
    set(OpenCV_dep OpenCV)
    list(APPEND YARP_COMPONENTS cv)
endif()

#YARP
find_package(YARP COMPONENTS ${YARP_COMPONENTS} REQUIRED)
yarp_configure_external_installation(event-driven)

#metavision packages (prophesee is old version)
find_package(MetavisionSDK COMPONENTS core driver QUIET)
if(MetavisionSDK_FOUND)
  message(STATUS "Found MetavisionSDK: (found version ${MetavisionSDK_VERSION})")
endif()
find_package(prophesee_core QUIET)
if(prophesee_core_FOUND)
  message(STATUS "Found prophesee_core: (found version ${prophesee_core_VERSION})")
endif()

#build the library
add_subdirectory(ev2)

#build additional tools
add_subdirectory(cpp_tools)

#install the package
include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            EXPORT eventdriven
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY SameMajorVersion
                            LOWERCASE_FILENAMES
                            NAMESPACE ev::
                            DEPENDENCIES "YARP COMPONENTS os sig" ${OpenCV_dep}
                            INCLUDE_CONTENT "yarp_configure_external_installation(event-driven NO_PATH_D)")

include(AddUninstallTarget)
