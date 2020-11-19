# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(vSpinnakerEventsMapper)

include(GNUInstallDirs)
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

find_package(YCM REQUIRED)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)


find_package(YARP COMPONENTS OS sig math dev REQUIRED)
find_package(event-driven REQUIRED)

#default the install location to that of event-driven

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

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini)
endif()
             
#uninstall target?
