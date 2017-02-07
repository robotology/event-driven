cmake_minimum_required(VERSION 2.6)

if(V_ADVMODULES)

  add_subdirectory(vCluster)
  #add_subdirectory(vCorner)
  add_subdirectory(vRepTest)

  if(ICUB_FOUND)
      #add_subdirectory(vCircleDisparity)
      add_subdirectory(vParticleFilter)
  endif(ICUB_FOUND)

endif(V_ADVMODULES)

add_subdirectory(vFlow)
if(ICUB_FOUND)
  add_subdirectory(vCircle)
endif(ICUB_FOUND)
