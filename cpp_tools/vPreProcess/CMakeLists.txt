project(vPreProcess)

add_executable(${PROJECT_NAME} vPreProcess.cpp vision.h skin.h imu.h audio.h)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              ${OpenCV_LIBRARIES}
                                              ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})


