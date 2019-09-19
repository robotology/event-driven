# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(example-module)

#include(GNUInstallDirs)
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()


# mandatory use of these packages
find_package(YARP COMPONENTS OS sig math dev REQUIRED)
find_package(event-driven REQUIRED)

#default the install location to that of event-driven

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

#install .ini files and .xml.template files to share event-driven
yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES app_${PROJECT_NAME}.xml 
             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_INSTALL_DIR})
             
#uninstall target?
