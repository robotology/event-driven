project(EDPR_imutk_app)

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Boost QUIET)
if(NOT Boost_FOUND)
  message("Boost not found - not compiling IMU calibration")
  return()
endif()

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  message("Eigen3 not found - not compiling IMU calibration")
  return()
endif()

find_package(Ceres QUIET)
if(NOT Ceres_FOUND)
  message("Ceres not found - not compiling IMU calibration")
  return()
endif()

include_directories(./include
                    /usr/include
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})

#Vis3D
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS}  
                     ${GLUT_INCLUDE_DIRS}
                     ${CMAKE_CURRENT_BINARY_DIR} )
                     
set(IMU_TK_VIS_EXTRA_SOURCES 
    /usr/local/src/imu_tk/src/vis_extra/gl_camera.cpp 
    /usr/local/src/imu_tk/src/vis_extra/opengl_3d_scene.cpp )
set(IMU_TK_VIS_EXTRA_HEADERS
  /usr/local/src/imu_tk/include/imu_tk/vis_extra/opengl_3d_scene.h )
    
qt4_wrap_cpp(IMU_TK_VIS_EXTRA_HEADERS_MOC ${IMU_TK_VIS_EXTRA_HEADERS})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

aux_source_directory( /usr/local/src/imu_tk/src IMU_TK_CORE_SRC )
set( IMU_TK_SRC ${IMU_TK_CORE_SRC} ${IMU_TK_VIS_EXTRA_HEADERS_MOC} ${IMU_TK_VIS_EXTRA_SOURCES} )

set (IMU_TK_INCLUDE_DIR /usr/local/src/imu_tk/include CACHE STRING "imu_tk include directories")
set (IMU_TK_LIB_DIR /usr/local/src/imu_tk/lib CACHE STRING "imu_tk libraries directories")
set (IMU_TK_LIBS imu_tk ${CERES_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} 
     CACHE STRING "imu_tk libraries")

#add_library( imu_tk UNKNOWN IMPORTED)
#set_property(TARGET imu_tk PROPERTY IMPORTED_LOCATION "/usr/local/src/imu_tk/lib/libimu_tk.a")

add_library(imu_tk ${IMU_TK_SRC})
set_target_properties(imu_tk PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${IMU_TK_LIB_DIR})
target_include_directories(imu_tk PRIVATE ${IMU_TK_INCLUDE_DIR})

message( "${IMU_TK_LIBS}" )

add_executable(EDPR_imu_calib EDPR_imu_calib.cpp)
target_include_directories(EDPR_imu_calib PRIVATE ${IMU_TK_INCLUDE_DIR})
target_link_libraries( EDPR_imu_calib ${IMU_TK_LIBS})
#set_target_properties( EDPR_imu_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

