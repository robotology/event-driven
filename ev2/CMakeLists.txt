set( folder_source
  event-driven/core/codec.cpp
  event-driven/core/comms.cpp
  #include/event-driven/core/vPort.cpp
  event-driven/core/utilities.cpp
)

set(public_header_files event-driven/core.h)
set( folder_header
  event-driven/core.h
  event-driven/core/codec.h
  event-driven/core/utilities.h
  event-driven/core/comms.h
  #include/event-driven/core/vPort.h
)

if(OpenCV_FOUND)

  list( APPEND folder_source 
    event-driven/vis/filters.cpp
    event-driven/vis/IPT.cpp
    event-driven/vis/draw.cpp
    event-driven/algs/surface.cpp
    event-driven/algs/corner.cpp
    event-driven/algs/flow.cpp
    #include/event-driven/vis/vDraw_basic.cpp
    #include/event-driven/vis/vDraw_ISO.cpp
    #src/vDraw_skin.cpp
  )
  
  list( APPEND public_header_files event-driven/vis.h
                                   event-driven/algs.h)
  list( APPEND folder_header 
    event-driven/vis.h
    event-driven/vis/filters.h
    event-driven/vis/IPT.h
    event-driven/vis/draw.h
    event-driven/algs.h
    event-driven/algs/surface.h
    event-driven/algs/corner.h
    event-driven/algs/flow.h
    #include/event-driven/vis/vDraw.h
    #include/event-driven/vDrawSkin.h
  )

endif()

# include_directories(algorithms)


# Create everything needed to build our library
add_library(${EVENTDRIVEN_LIBRARY} ${folder_source} ${folder_header})
add_library(ev::${EVENTDRIVEN_LIBRARY} ALIAS ${EVENTDRIVEN_LIBRARY})


set_target_properties(${EVENTDRIVEN_LIBRARY} PROPERTIES PUBLIC_HEADER "${public_header_files}"
                                                          VERSION ${event-driven_VERSION}
                                                          SOVERSION 2)
target_include_directories(${EVENTDRIVEN_LIBRARY} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>") # FIXME INSTALL PATH

target_compile_definitions(${EVENTDRIVEN_LIBRARY} PRIVATE 
    CLOCK_PERIOD=${VLIB_CLOCK_PERIOD_NS}
    TIMER_BITS=${VLIB_TIMER_BITS}
    $<$<BOOL:${VLIB_ENABLE_TS}>:ENABLE_TS=1>)

target_compile_options(${EVENTDRIVEN_LIBRARY} PRIVATE -Wall)

if(OpenCV_FOUND)
    target_link_libraries(${EVENTDRIVEN_LIBRARY} PUBLIC YARP::YARP_os
                                                        YARP::YARP_sig
                                                        pthread
                                                        ${OpenCV_LIBRARIES})
else()
    target_link_libraries(${EVENTDRIVEN_LIBRARY} PUBLIC YARP::YARP_os
                                                        YARP::YARP_sig
                                                        pthread)
endif()

install(TARGETS ${EVENTDRIVEN_LIBRARY}
        EXPORT eventdriven
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${EVENTDRIVEN_LIBRARY}" COMPONENT dev)

install(DIRECTORY "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/event-driven" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" FILES_MATCHING PATTERN "*.h")
