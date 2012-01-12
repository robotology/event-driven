# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "dvsIntegrator")

PROJECT(${KEYWORD})

#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIR}    
    ${EMORPHLIB_INCLUDE_DIRS}	
    )

IF(NOT IPP_INCLUDE_DIRS)
	MESSAGE(ERROR ": No IPP_INCLUDE_DIR found. Please check it out")	
ENDIF(NOT IPP_INCLUDE_DIRS)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(ImageProcessModuleSRC
    src/ImageProcessModule.cpp
)


LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}

)


IF (WIN32) #required in win
  LINK_LIBRARIES(
    ${OpenCV_LIBRARIES}
    ${IDL_LIBRARIES}
    ${IPP_LIBRARIES}  
    ${IKIN_LIBRARIES}
)	
ELSE(WIN32) #required in linux
  LINK_LIBRARIES(
    ${OpenCV_LIBRARIES}
    ${IDL_LIBRARIES}
    ${IPP_LIBRARIES} 	
    ${IKIN_LIBRARIES}
    pthread
)
ENDIF (WIN32)



# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


INSTALL_TARGETS(/lib ${KEYWORD})
INSTALL_TARGETS(/bin ${KEYWORD})

