option(ENABLE_zynqgrabber "Build zynqGrabber" OFF)
option(ENABLE_chronocamgrabber "Build chronocamGrabber" OFF)

if(ENABLE_zynqgrabber)
    add_subdirectory(zynqGrabber)
endif(ENABLE_zynqgrabber)

if(ENABLE_chronocamgrabber)
    add_subdirectory(chronocamGrabber)
endif(ENABLE_chronocamgrabber)

