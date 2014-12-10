CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME "MTHeading")
project(${PROJECT_NAME})

FIND_PACKAGE(YARP)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
	   	    ${EMORPHLIB_INCLUDE_DIRS}
)


FILE (GLOB source_folder src/*.cpp)
FILE (GLOB header_folder include/iCub/*.h)

add_executable(${PROJECT_NAME} ${source_folder} ${header_folder})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YARP_LIBRARIES})

INSTALL_TARGETS(/bin ${PROJECT_NAME})

