#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# After change in directory structure the uncommented modules were moved from src to /src/processing/ : 26 june 2014 by Himanshu Akolkar

cmake_minimum_required(VERSION 2.6)



IF(NOT WIN32) 
#	add_subdirectory   (  aexGrabber                   )		
#	add_subdirectory   (  asvGrabber                   )
#	add_subdirectory   (  basicAexGrabber              )
ENDIF (NOT WIN32)

#add_subdirectory  ( angle2Sac   )
#add_subdirectory  (  buffer2image		)
#add_subdirectory  (  disparitySelector		)
#add_subdirectory  ( eventSelectiveAttention )
#add_subdirectory  ( eventFeatureExtractor    )
#add_subdirectory  ( harmonicPathPlanner  )
#add_subdirectory  ( upSampler    )
#add_subdirectory  ( velocityExtractor    )
#add_subdirectory  ( visualAligner    )
#add_subdirectory  ( eventBottleProcessor )
add_subdirectory  ( eventBottleConverter    )
add_subdirectory  ( eventAnalysis    )
add_subdirectory  (cBottle2eBottle          )

# --------  OPTICFLOW SUBFOLDERS ---------------------
#add_subdirectory   (  eventOpticalFlow             )

# --------  TIMESTAMP-BASED OPTICAL FLOW -------------
#add_subdirectory   (  tsOptFlow                    )
#add_subdirectory   (  tsOptFlowViewer              )

# --------  OBJECT RECOGNITION
#add_subdirectory   ( recognitionModules/extraction )
#add_subdirectory   ( recognitionModules/association)
#add_subdirectory   ( recognitionModules/recognition)
#add_subdirectory   ( recognitionModules/aggregation)

#----------- Clustering -----------------------------
#add_subdirectory   ( clustering                     )

if(ICUB_USE_IPOPT)
        #add_subdirectory ( targetFinder                  )
	#add_subdirectory (  wingsTranslator              )
endif(ICUB_USE_IPOPT)

