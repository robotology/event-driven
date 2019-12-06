
# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

project(vGazeDemo)

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

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

set(CMAKE_INSTALL_BINDIR /usr/local/src/robot/iCubContrib/bin)

add_executable(${PROJECT_NAME} ${source} ${header})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_math
                                              ev::event-driven)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

#yarp_install(FILES ${PROJECT_NAME}.ini
#             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
#yarp_install(FILES app_${PROJECT_NAME}.xml.template 
#             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_TEMPLATES_INSTALL_DIR})

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini ${PROJECT_NAME}.xml app_${PROJECT_NAME}.xml.template leftScope.xml rightScope.xml)
endif()

