CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME synapse)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
LINK_LIBRARIES(${YARP_LIBRARIES})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

#SET(GSL_DIR "$ENV{YARP_ROOT}/../conf" CACHE LOCATION "where is yarp?")
find_package(GSL REQUIRED)
LINK_LIBRARIES(${GSL_LIBRARIES})
INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS})

#FIND_PACKAGE(MATRIXPP REQUIRED)
#LINK_LIBRARIES(${MATRIXPP_LIBRARIES})
#INCLUDE_DIRECTORIES(${MATRIXPP_INCLUDE_DIRS})

#FIND_PACKAGE(VECTORPP REQUIRED)
#LINK_LIBRARIES(${VECTORPP_LIBRARIES})
#INCLUDE_DIRECTORIES(${VECTORPP_INCLUDE_DIRS})

#FIND_PACKAGE(ELUCASKANADE REQUIRED)
LINK_LIBRARIES(eLucasKanade)
INCLUDE_DIRECTORIES(${eLucasKanade_INCLUDE_DIRS})

#FIND_PACKAGE(VECBUFFER REQUIRED)
LINK_LIBRARIES(vecBuffer)
INCLUDE_DIRECTORIES(${vecBuffer_INCLUDE_DIRS})

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# Create everything needed to build our executable.
ADD_LIBRARY(${PROJECTNAME} STATIC ${folder_source} ${folder_header})

#ADD_LIBRARY(${PROJECTNAME} SHARED ${folder_source} ${folder_header})
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

#INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)
#INSTALL(FILES ${folder_header} DESTINATION include/iCub)

# Makes VS a bit quiter wrt warnings.
IF(MSVC)
    SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

# add functionalities like icub_add_library/icub_add_executable
#include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
MESSAGE(STATUS "icub_dir: ${ICUB_DIR}")
include(${ICUB_DIR}/../conf/iCubHelpers.cmake)
SET(PROJECT_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
                    
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
                                   DESTINATION include/iCub
                                   FILES ${folder_header})


