
project(dv-bridge-sdk)

add_executable(${PROJECT_NAME} dv-bridge-sdk.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven
                                              dv::processing
                                              dl)                                             

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<BOOL:${VLIB_ENABLE_TS}>:ENABLE_TS=1>)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
