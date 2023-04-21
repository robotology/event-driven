project(zynqGrabber2)

#file(GLOB source *.cpp)
#file(GLOB header *.h)

add_executable(${PROJECT_NAME} zynqModule.cpp hpuInterface.cpp hpuInterface.h)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-overflow)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              ev::${EVENTDRIVEN_LIBRARY})

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<BOOL:${VLIB_ENABLE_TS}>:ENABLE_TS=1>)

#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

