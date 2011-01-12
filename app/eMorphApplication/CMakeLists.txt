########## application
set(appname eMorphApplication)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

icub_app_install(${appname}
				FILES ${conf}
				DESTINATION conf)
icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts/)
