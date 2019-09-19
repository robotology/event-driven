cmake_minimum_required(VERSION 3.5)

project(zynqGrabber)
add_compile_options(-Wno-overflow)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
file(GLOB biasfiles bias_files/*.ini)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${source} ${header})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARY})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

yarp_install(FILES ${PROJECT_NAME}.ini ${biasfiles} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(ADD_DOCS_TO_IDE)
    add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini ${PROJECT_NAME}.xml ${biasfiles})
endif(ADD_DOCS_TO_IDE)
