project(zynqGrabber)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
file(GLOB biasfiles bias_files/*.ini)

add_executable(${PROJECT_NAME} ${source} ${header})

message(STATUS "${PROJECT_SOURCE_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-overflow)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              ev::${EVENTDRIVEN_LIBRARY})

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<BOOL:${VLIB_ENABLE_TS}>:ENABLE_TS=1>)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

yarp_install(FILES ${PROJECT_NAME}.ini
             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})

