project(zynqGrabber)
add_compile_options(-Wno-overflow)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
file(GLOB biasfiles bias_files/*.ini)

add_executable(${PROJECT_NAME} ${source} ${header})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini ${PROJECT_NAME}.xml ${biasfiles})
endif(ADD_DOCS_TO_IDE)

