project(ev-calibrate)

add_executable(${PROJECT_NAME}-mono calibrate.cpp)

target_include_directories(${PROJECT_NAME}-mono PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-mono PRIVATE YARP::YARP_os
                                               YARP::YARP_init
                                               ${OpenCV_LIBRARIES}
                                               ev::${EVENTDRIVEN_LIBRARY})
install(TARGETS ${PROJECT_NAME}-mono DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(${PROJECT_NAME}-stereo stereo-calibrate.cpp)
target_include_directories(${PROJECT_NAME}-stereo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-stereo PRIVATE YARP::YARP_os
                                               YARP::YARP_init
                                               ${OpenCV_LIBRARIES}
                                               ev::${EVENTDRIVEN_LIBRARY})
install(TARGETS ${PROJECT_NAME}-stereo DESTINATION ${CMAKE_INSTALL_BINDIR})

