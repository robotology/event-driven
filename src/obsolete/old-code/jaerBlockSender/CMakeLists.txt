CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAMELOCAL jaerBlockSender)
PROJECT(${PROJECTNAMELOCAL})

FIND_PACKAGE(YARP REQUIRED)
#FIND_PACKAGE(EVENTSUNMASK REQUIRED)
#FIND_PACKAGE(EVENTBUFFER REQUIRED)
#FIND_PACKAGE(EMORPHLIB)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
FILE(GLOB folder_source source/*.cpp)
FILE(GLOB folder_header include/*.hpp)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${EVENTBUFFER_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EMORPHLIB_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${EVENTSUNMASK_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAMELOCAL}   ${folder_source} 
                                ${folder_header})

TARGET_LINK_LIBRARIES (	${PROJECTNAMELOCAL}
						${YARP_LIBRARIES}
                        #${EVENTBUFFER_LIBRARIES}
                        #${EMORPHLIB_LIBRARIES}
                        #${EVENTSUNMASK_LIBRARIES}
                        eMorphLib
						)

#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)# Create everything needed to build our executable.
#SET(EXECUTABLE_OUTPUT_PATH $ENV{MODULES_ROOT}/bin)# Create everything needed to build our executable.
INSTALL_TARGETS(/bin ${PROJECTNAMELOCAL})

# Makes VS a bit quiter wrt warnings.
#IF(MSVC)
#    SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
#ENDIF(MSVC)
