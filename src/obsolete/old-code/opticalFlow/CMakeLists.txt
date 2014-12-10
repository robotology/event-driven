##################
# IMPORTANT: the order with which you add directories 
# here matters. 
# Put libraries in dependency order. If A depends from B
# put B before A. Failure to do so might break the build.
# 

add_subdirectory(eventBuffer)
add_subdirectory(vecBuffer)
add_subdirectory(eLucasKanade)
add_subdirectory(synapse)
add_subdirectory(readRaw)
add_subdirectory(viewer)