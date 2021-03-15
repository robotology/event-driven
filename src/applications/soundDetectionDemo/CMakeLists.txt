# project(soundDetectionDemo)

# vAuditiory Attention
add_executable( vAuditoryAttention src/vAuditoryAttention.cpp)
target_link_libraries(vAuditoryAttention PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_math
                                              ev::event-driven)
install(TARGETS vAuditoryAttention DESTINATION ${CMAKE_INSTALL_BINDIR})

# vCochleaEventsMapper
add_executable( vCochleaEventsMapper src/vCochleaEventsMapper.cpp)
target_link_libraries(vCochleaEventsMapper PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              ev::event-driven)
install(TARGETS vCochleaEventsMapper DESTINATION ${CMAKE_INSTALL_BINDIR})

# robotMovement
add_executable( vRobotMovement src/vRobotMovement.cpp)
target_link_libraries(vRobotMovement PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_math
                                              ev::event-driven)
install(TARGETS vRobotMovement DESTINATION ${CMAKE_INSTALL_BINDIR})

# soundClassification
add_executable( vSoundClassification src/vSoundClassification.cpp)
target_link_libraries(vSoundClassification PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_math
                                              ev::event-driven)
install(TARGETS vSoundClassification DESTINATION ${CMAKE_INSTALL_BINDIR})

# spinnakerEventsMapper
add_executable( vSpinnakerEventsMapper src/vSpinnakerEventsMapper.cpp)
target_link_libraries(vSpinnakerEventsMapper PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_math
                                              ev::event-driven)
install(TARGETS vSpinnakerEventsMapper DESTINATION ${CMAKE_INSTALL_BINDIR})

yarp_install(FILES  config/vSpinnakerEventsMapper.ini config/vSoundClassification.ini 
                    config/vRobotMovement.ini config/vAuditoryAttention.ini config/vCochleaEventsMapper.ini
                    DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
#yarp_install(FILES ${PROJECT_NAME}.ini
#             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${CONTEXT_DIR})
#yarp_install(FILES app_${PROJECT_NAME}.xml.template 
#             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_TEMPLATES_INSTALL_DIR})


