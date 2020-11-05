project(cochlea-viewer-example)

yarp_install(FILES app_${PROJECT_NAME}.xml
             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_INSTALL_DIR})

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES app_${PROJECT_NAME}.xml)
endif()

