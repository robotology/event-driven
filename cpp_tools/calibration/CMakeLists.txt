project(ev-calibrate)

add_executable(${PROJECT_NAME} calibrate.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                               YARP::YARP_init
                                               ${OpenCV_LIBRARIES}
                                               ev::${EVENTDRIVEN_LIBRARY})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(stereo-${PROJECT_NAME} stereo-calibrate.cpp)
target_include_directories(stereo-${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo-${PROJECT_NAME} PRIVATE YARP::YARP_os
                                               YARP::YARP_init
                                               ${OpenCV_LIBRARIES}
                                               ev::${EVENTDRIVEN_LIBRARY})
install(TARGETS stereo-${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

