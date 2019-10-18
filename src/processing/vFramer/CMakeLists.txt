project(vFramer)
set(PROJECT_ALIAS vFramerLite)

file(GLOB source src/vFramerLite.cpp src/vDraw.cpp src/*_drawers.cpp)
file(GLOB header include/vFramerLite.h include/vDraw.h)

add_executable(${PROJECT_ALIAS} ${source} ${header})

target_include_directories(${PROJECT_ALIAS} PRIVATE ${PROJECT_SOURCE_DIR}/include
                                                   ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_ALIAS} PRIVATE YARP::YARP_OS
                                               YARP::YARP_init
                                               YARP::YARP_sig
                                               YARP::YARP_cv
                                               ${OpenCV_LIBRARIES}
                                               ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_ALIAS} DESTINATION ${CMAKE_INSTALL_BINDIR})

yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini ${PROJECT_NAME}.xml)
endif(ADD_DOCS_TO_IDE)

