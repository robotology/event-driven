cmake_minimum_required(VERSION 2.6)

option(BUILD_APPLICATIONS "Build event-driven applications" OFF)
option(BUILD_HARDWAREIO "Build event-driven hardware interfaces" OFF)
option(BUILD_PROCESSING "Build event-driven processing modules" OFF)

if(BUILD_APPLICATIONS)
    add_subdirectory(applications)
endif(BUILD_APPLICATIONS)

if(BUILD_HARDWAREIO)
    add_subdirectory(hardwareio)
endif(BUILD_HARDWAREIO)

if(BUILD_PROCESSING)
    add_subdirectory(processing)
endif(BUILD_PROCESSING)
