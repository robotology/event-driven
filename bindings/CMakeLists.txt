cmake_minimum_required(VERSION 3.5)

option(BUILD_BINDINGS "Build python bindings" OFF)

if(BUILD_BINDINGS)
    find_package(SWIG REQUIRED)
    include(${SWIG_USE_FILE})
    find_package(PythonLibs)

    file(GLOB source *.i)

    include_directories(${EVENTDRIVENLIBS_INCLUDE_DIRS}
                        ${PYTHON_INCLUDE_PATH}
                        ${CMAKE_CURRENT_SOURCE_DIR})
    set(CMAKE_SWIG_FLAGS "")
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bindings )
    set_source_files_properties(event_driven.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(event_driven.i PROPERTIES SWIG_FLAGS "-includeall")

    swig_add_library(event_driven LANGUAGE python SOURCES event_driven.i)
    swig_link_libraries(event_driven ${PYTHON_LIBRARIES} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})

    execute_process(COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
    #INSTALL ( FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/_event_driven.so DESTINATION ${PYTHON_SITE_PACKAGES} )
    #INSTALL ( FILES ${CMAKE_CURRENT_BINARY_DIR}/event_driven.py DESTINATION ${PYTHON_SITE_PACKAGES} )


endif(BUILD_BINDINGS)

