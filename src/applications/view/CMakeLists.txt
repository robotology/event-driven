cmake_minimum_required(VERSION 2.6)

set(MODULENAME vView)
project(${MODULENAME})

yarp_install(FILES app_${MODULENAME}.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
if(USE_QTCREATOR)
    add_custom_target(${MODULENAME}_token SOURCES app_${MODULENAME}.xml)
endif(USE_QTCREATOR)
