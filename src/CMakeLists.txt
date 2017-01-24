cmake_minimum_required(VERSION 2.6)

if(NOT V_BUILD_ZYNQ)

    add_subdirectory(core_modules)
    file(GLOB xmldocumentation ${xmldocumentation} ${CMAKE_CURRENT_SOURCE_DIR}/core_modules/*/*.xml)

    if(V_ADVMODULES)
      add_subdirectory(modules)
      file(GLOB xmldocumentation ${xmldocumentation} ${CMAKE_CURRENT_SOURCE_DIR}/modules/*/*.xml)
    endif(V_ADVMODULES)

    if(ICUB_FOUND)
        add_subdirectory(robotio)
        file(GLOB xmldocumentation ${xmldocumentation} ${CMAKE_CURRENT_SOURCE_DIR}/robotio/*/*.xml)
    endif(ICUB_FOUND)

endif(NOT V_BUILD_ZYNQ)

if(V_USE_QTCREATOR)
    #add_custom_target(doctoken SOURCES ${xmldocumentation})
    add_subdirectory(hardwareio)
endif(V_USE_QTCREATOR)

file(GLOB xmldocumentation ${CMAKE_CURRENT_SOURCE_DIR}/hardwareio/*/*.xml)
