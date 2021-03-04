
project(atis-bridge-sdk)

add_executable(${PROJECT_NAME} atis-bridge-sdk.cpp)

if(prophesee_core_FOUND)
target_compile_definitions(${PROJECT_NAME} PRIVATE prophesee_core_FOUND)
target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven
                                              prophesee_core
                                              prophesee_driver)
else()
target_compile_definitions(${PROJECT_NAME} PRIVATE MetavisionSDK_FOUND)
target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven
                                              MetavisionSDK::core
                                              MetavisionSDK::driver)                                             
endif()
                                              

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
