#################################################################################
#                                                                               #
# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)           #
# All Rights Reserved.                                                          #
#                                                                               #
#################################################################################

# @authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>

project(IMUCalibDumper)

add_executable(${PROJECT_NAME} IMUCalibDumper.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_math
                                              ev::event-driven
                                              stdc++fs)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
