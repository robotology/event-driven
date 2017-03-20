cmake_minimum_required(VERSION 2.6)

add_subdirectory(vCluster)
#add_subdirectory(vCorner)
add_subdirectory(vRepTest)
add_subdirectory(vFlow)

if(ICUB_FOUND)
    #add_subdirectory(vCircleDisparity)
    add_subdirectory(vParticleFilter)
    add_subdirectory(vCircle)
endif(ICUB_FOUND)

