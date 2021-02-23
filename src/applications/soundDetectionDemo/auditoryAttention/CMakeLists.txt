# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(vAuditoryAttention)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

add_executable(${PROJECT_NAME} ${source} ${header})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

#install .ini files and .xml.template files to share event-driven
yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES app_${PROJECT_NAME}.xml 
             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_INSTALL_DIR})

option(ADD_DOCS_TO_IDE "Add apps/documentation to IDE" ON)
if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES app_${PROJECT_NAME}.xml ${PROJECT_NAME}.ini README.md)
endif()
             
#uninstall target?
