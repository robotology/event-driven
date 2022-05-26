project(esim-yarp)

add_executable(${PROJECT_NAME} src/event_simulator.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_cv
        ${OpenCV_LIBRARIES}
        ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

# yarp_install(FILES ${PROJECT_NAME}.ini
#         DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})

if(ADD_DOCS_TO_IDE)
    add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini ${PROJECT_NAME}.xml)
endif(ADD_DOCS_TO_IDE)
