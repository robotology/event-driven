CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME tutorials)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})
	
## this is for testing only, please ignore if you are reading tutorials				
INCLUDE(CTest)
### end test specific code

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")					

# Create everything needed to build our executable.

ADD_EXECUTABLE(tutorial_eyeControl tutorial_eyeControl.cpp)
TARGET_LINK_LIBRARIES(tutorial_eyeControl ${YARP_LIBRARIES})



## again this is for testing only, please ignore if you are reading tutorials
#ADD_EXECUTABLE(test_icubVis smoke-tests/icubVis.cpp)
#TARGET_LINK_LIBRARIES(test_icubVis ${YARP_LIBRARIES} iCubVis)

IF(ICUB_USE_IPOPT)
    ADD_EXECUTABLE(test_ipopt smoke-tests/ipopt.cpp)
    TARGET_LINK_LIBRARIES(test_ipopt ${YARP_LIBRARIES} iKin)
ENDIF()

### execute tests
GET_TARGET_PROPERTY(EXELOC test_icubVis LOCATION_RELEASE)
ADD_TEST(test_icubVis ${EXELOC})

IF(ICUB_USE_IPOPT)
    GET_TARGET_PROPERTY(EXELOC test_ipopt LOCATION_RELEASE)
    ADD_TEST(test_ipopt ${EXELOC})
ENDIF()
###################### end of test specific code


INSTALL_TARGETS(/bin tutorial_eyeControl)
