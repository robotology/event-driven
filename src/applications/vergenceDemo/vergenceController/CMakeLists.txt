project(vergenceController)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

add_executable(${PROJECT_NAME} ${source} ${header})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              ev::${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

file(GLOB xmls *.xml)

yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
yarp_install(FILES app_${PROJECT_NAME}.xml.template 
             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_TEMPLATES_INSTALL_DIR})

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini ${xmls} ${PROJECT_NAME}.xml app_${PROJECT_NAME}.xml.template)
endif()


