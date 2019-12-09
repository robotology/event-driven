option(ENABLE_zynqgrabber "Build zynqGrabber" OFF)
option(ENABLE_chronocamgrabber "Build chronocamGrabber" OFF)
option(ENABLE_binarydumper "Build binaryDumper" OFF)

if(ENABLE_zynqgrabber)
    add_subdirectory(zynqGrabber)
endif()

if(ENABLE_chronocamgrabber)
    add_subdirectory(chronocamGrabber)
endif()

if(ENABLE_binarydumper)
    add_subdirectory(binaryDumper)
endif()
