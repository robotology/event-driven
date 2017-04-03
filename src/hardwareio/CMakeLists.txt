cmake_minimum_required(VERSION 2.6)

option(ENABLE_spinterface "Build the SpiNNaker Ethernet Interface" OFF)
option(ENABLE_zynqgrabber "Build zynqGrabber" OFF)

if(ENABLE_spinterface)
    add_subdirectory(spinterface)
endif(ENABLE_spinterface)

if(ENABLE_zynqgrabber)
    add_subdirectory(zynqGrabber)
endif(ENABLE_zynqgrabber)

