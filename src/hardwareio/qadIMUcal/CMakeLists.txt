project(qadIMUcal)

add_executable(${PROJECT_NAME} qadIMUcal.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

