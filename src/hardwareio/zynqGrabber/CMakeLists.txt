cmake_minimum_required(VERSION 2.6)

set(MODULENAME zynqGrabber)
project(${MODULENAME})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
file(GLOB biasfiles bias_files/*.ini)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

if(VLIB_10BITCODEC)
    set_target_properties(${MODULENAME} PROPERTIES COMPILE_DEFINITIONS TENBITCODEC)
endif(VLIB_10BITCODEC)

install(TARGETS ${MODULENAME} DESTINATION bin)

yarp_install(FILES ${MODULENAME}.ini ${biasfiles} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES ${MODULENAME}.ini ${MODULENAME}.xml ${biasfiles})
endif(USE_QTCREATOR)
