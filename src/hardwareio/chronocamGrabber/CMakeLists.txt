cmake_minimum_required(VERSION 3.5)

project(chronocamGrabber)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
file(GLOB biasfiles bias_files/*.ini)


find_package(libCCamDevice REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/include
                    ${EVENTDRIVENLIBS_INCLUDE_DIRS}
                    ${LIBCCAMDEVICE_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${source} ${header})

target_link_libraries(${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES} ${LIBCCAMDEVICE_LIBRARIES})

#add_definitions("-D${VLIB_CODEC_TYPE}")

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

yarp_install(FILES ${PROJECT_NAME}.ini ${biasfiles} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
if(USE_QTCREATOR)
    add_custom_target(${PROJECT_NAME}_token SOURCES ${PROJECT_NAME}.ini ${PROJECT_NAME}.xml ${biasfiles})
endif(USE_QTCREATOR)
