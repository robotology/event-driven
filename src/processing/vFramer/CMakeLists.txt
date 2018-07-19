cmake_minimum_required(VERSION 2.6)

set(MODULENAME vFramer)
project(${MODULENAME})

file(GLOB source src/vFramer.cpp src/vDraw.cpp src/*_draw.cpp )
file(GLOB header include/vFramer.h include/vDraw.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

install(TARGETS ${MODULENAME} DESTINATION bin)

yarp_install(FILES ${MODULENAME}.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES ${MODULENAME}.ini ${MODULENAME}.xml)
endif(USE_QTCREATOR)

set(MODULENAME vFramerLite)
project(${MODULENAME})

file(GLOB source src/vFramerLite.cpp src/vDraw.cpp src/*_draw.cpp )
file(GLOB header include/vFramerLite.h include/vDraw.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${MODULENAME} ${source} ${header})

target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

install(TARGETS ${MODULENAME} DESTINATION bin)

#yarp_install(FILES ${MODULENAME}.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
#if(USE_QTCREATOR)
#    add_custom_target(${MODULENAME}_token SOURCES ${MODULENAME}.ini ${MODULENAME}.xml)
#endif(USE_QTCREATOR)
