cmake_minimum_required(VERSION 2.6)

set(MODULENAME vAnnotation)
project(${MODULENAME})

include_directories(${PROJECT_SOURCE_DIR}/include
        ${YARP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EVENTDRIVENLIBS_INCLUDE_DIRS})

yarp_install(FILES app_${MODULENAME}.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES ${MODULENAME}.ini ${MODULENAME}.xml)
endif(USE_QTCREATOR)
