project(vPreProcess)

add_executable(${PROJECT_NAME} vPreProcess.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              ${OpenCV_LIBRARIES}
                                              ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})

