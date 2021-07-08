#choose clock rate of FPGA
set(VLIB_CLOCK_PERIOD_NS 80 CACHE STRING "event timestamp clock period (ns)")

#choose number of bits of the FPGA timer to use
set(VLIB_TIMER_BITS 30 CACHE STRING "event timestamp maximum = 2^TIMERBITS")
string(COMPARE GREATER ${VLIB_TIMER_BITS} 31 TOOMANYBITSINCOUNTER)
if(TOOMANYBITSINCOUNTER)
  message( FATAL_ERROR "Maximum bits useable is 31 (VLIB_TIMER_BITS)")
endif()

set( folder_source
  src/core/comms.cpp
  #include/event-driven/core/vPort.cpp
  src/core/utilities.cpp
)

set( folder_header
  include/event-driven/core.h
  include/event-driven/core/utilities.h
  include/event-driven/core/comms.h
  #include/event-driven/core/vPort.h
)

if(OpenCV_FOUND)

  list( APPEND folder_source 
    src/vis/filters.cpp
    src/vis/IPT.cpp
    #include/event-driven/vis/vDraw_basic.cpp
    #include/event-driven/vis/vDraw_ISO.cpp
    #src/vDraw_skin.cpp
  )

  list( APPEND folder_header 
    include/event-driven/vis.h
    include/event-driven/vis/filters.h
    include/event-driven/vis/IPT.h              
    #include/event-driven/vis/vDraw.h
    #include/event-driven/vDrawSkin.h
  )

endif()

# include_directories(algorithms)


# Create everything needed to build our library
add_library(${EVENTDRIVEN_LIBRARY} ${folder_source} ${folder_header})
add_library(ev::${EVENTDRIVEN_LIBRARY} ALIAS ${EVENTDRIVEN_LIBRARY})


set_target_properties(${EVENTDRIVEN_LIBRARY} PROPERTIES PUBLIC_HEADER "${folder_header}"
                                                          VERSION ${event-driven_VERSION}
                                                          SOVERSION 2)
target_include_directories(${EVENTDRIVEN_LIBRARY} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>") # FIXME INSTALL PATH

target_compile_definitions(${EVENTDRIVEN_LIBRARY} PRIVATE 
    CLOCK_PERIOD=${VLIB_CLOCK_PERIOD_NS}
    TIMER_BITS=${VLIB_TIMER_BITS})

target_compile_options(${EVENTDRIVEN_LIBRARY} PRIVATE -Wall)

if(OpenCV_FOUND)
    target_link_libraries(${EVENTDRIVEN_LIBRARY} PUBLIC YARP::YARP_os
                                                        YARP::YARP_sig
                                                        ${OpenCV_LIBRARIES})
else()
    target_link_libraries(${EVENTDRIVEN_LIBRARY} PUBLIC YARP::YARP_os
                                                        YARP::YARP_sig)
endif()

install(TARGETS ${EVENTDRIVEN_LIBRARY}
        EXPORT eventdriven
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${EVENTDRIVEN_LIBRARY}" COMPONENT dev)

