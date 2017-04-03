cmake_minimum_required(VERSION 2.6)

set(MODULENAME vFlow)
project(${MODULENAME})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

install(TARGETS ${MODULENAME} DESTINATION bin)

yarp_install(FILES ${MODULENAME}.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES ${MODULENAME}.ini ${MODULENAME}.xml)
endif(USE_QTCREATOR)
