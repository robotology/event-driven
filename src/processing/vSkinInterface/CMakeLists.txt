cmake_minimum_required(VERSION 2.6)

set(MODULENAME vSkinInterface)
project(${MODULENAME})

file(GLOB source src/module.cpp)
file(GLOB header include/module.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

install(TARGETS ${MODULENAME} DESTINATION bin)

yarp_install(FILES ${MODULENAME}.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(ADD_DOCS_TO_IDE)
    add_custom_target(${MODULENAME}_docs SOURCES ${MODULENAME}.ini ${MODULENAME}.xml)
endif(ADD_DOCS_TO_IDE)
