
project(atis-bridge-sdk)

add_executable(${PROJECT_NAME} atis-bridge-sdk.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven
                                              MetavisionSDK::core 
                                              MetavisionSDK::driver)


install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

