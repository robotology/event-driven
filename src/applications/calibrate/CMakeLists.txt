project(vCalib)

yarp_install(FILES app_${PROJECT_NAME}.xml.template app_${PROJECT_NAME}Chess.xml.template
             DESTINATION ${EVENT-DRIVEN_APPLICATIONS_TEMPLATES_INSTALL_DIR})

if(ADD_DOCS_TO_IDE)
  add_custom_target(${PROJECT_NAME}_docs SOURCES app_${PROJECT_NAME}.xml.template app_${PROJECT_NAME}Chess.xml.template)
endif(ADD_DOCS_TO_IDE)
