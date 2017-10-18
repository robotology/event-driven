cmake_minimum_required(VERSION 2.6)

set(MODULENAME dualCamTransform)
project(${MODULENAME})

yarp_install(FILES app_${MODULENAME}.xml.template DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES app_${MODULENAME}.xml.template)
endif(USE_QTCREATOR)
