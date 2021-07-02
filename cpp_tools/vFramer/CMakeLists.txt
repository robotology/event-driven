project(vFramer)

add_executable(${PROJECT_NAME} vFramer.cpp drawers.cpp drawers.h)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS} ${EVENTDRIVEN_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                               YARP::YARP_init
                                               YARP::YARP_sig
                                               YARP::YARP_cv
                                               ${OpenCV_LIBRARIES}
                                               ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})

