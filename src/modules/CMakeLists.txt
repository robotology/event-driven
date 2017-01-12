cmake_minimum_required(VERSION 2.6)

add_subdirectory(vFlow)
add_subdirectory(vCluster)
add_subdirectory(vCorner)
add_subdirectory(vRepTest)

if(ICUB_FOUND)
    add_subdirectory(vCircle)
    add_subdirectory(vCircleDisparity)
    add_subdirectory(vParticleFilter)
endif(ICUB_FOUND)
