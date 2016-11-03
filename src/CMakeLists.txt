cmake_minimum_required(VERSION 2.6)

add_subdirectory(hardwareio)

if(NOT V_BUILD_ZYNQ)
    add_subdirectory(modules)
    if(ICUB_FOUND)
        add_subdirectory(robotio)
    endif(ICUB_FOUND)
endif(NOT V_BUILD_ZYNQ)

