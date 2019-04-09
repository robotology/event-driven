cmake_minimum_required(VERSION 2.6)

set(MODULENAME vFramerLite)
project(${MODULENAME})

file(GLOB source src/vFramerLite.cpp src/vDraw.cpp src/*_drawers.cpp)
file(GLOB header include/vFramerLite.h include/vDraw.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

install(TARGETS ${MODULENAME} DESTINATION bin)

yarp_install(FILES vFramer.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(ADD_DOCS_TO_IDE)
    add_custom_target(${MODULENAME}_docs SOURCES vFramer.ini vFramer.xml)
endif(ADD_DOCS_TO_IDE)
