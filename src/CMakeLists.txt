#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)


IF(NOT WIN32) 
#	add_subdirectory   (  aexGrabber                   )		
#	add_subdirectory   (  asvGrabber                   )
#	add_subdirectory   (  basicAexGrabber              )
ENDIF (NOT WIN32)

add_subdirectory   (  development               )
add_subdirectory   (  grabbers                  )
add_subdirectory   (  interfaces		)
add_subdirectory   (  processing                )
add_subdirectory   (  visualization             )
add_subdirectory   (  motorActions		)

#add_subdirectory  (  angel2SAC                    )	#moved to Processing
#add_subdirectory  (  asvBiasInterface             )	#moved to Interfaces
#add_subdirectory  (  basicCartesianCollector      )	#moved to Visualization	
#add_subdirectory  (  biasInterface                )	#moved to Interfaces
#add_subdirectory  (  bottleRepeater               )	#moved to Development  	
#add_subdirectory  (  binFile2yarp                 )	#moved to Development  	
#add_subdirectory  (  buffer2image                 )	#moved to processing
#add_subdirectory  (  cartesian_frame_converter    )	#moved to Visualization	
#add_subdirectory  (  cartesian_frame_printer      )	#moved to Visualization	
#add_subdirectory  (  cartesianFrameCollector      )	#moved to Visualization	
#add_subdirectory  (  device2yarp                  )	#moved to Grabbers
#add_subdirectory  (  dvsGrabber                   )	#moved to Grabbers
#add_subdirectory  (  dvsCalibrator                )	#moved to Development  	 
#add_subdirectory  (  disparitySelector            )	#moved to processing 
#add_subdirectory  (  eventDriven                  )	#moved to Development  	 
#add_subdirectory  (  eventSniffer                 )	#moved to Grabbers 
#add_subdirectory  (  eventSelectiveAttention      )	#moved to processing
#add_subdirectory  (  emorphUnmaskTest             )	#moved to Development  	 
#add_subdirectory  (  eventProcessor               )	#moved to Development  	
#add_subdirectory  (  eventFeatureExtractor        )	#moved to Processing 
#add_subdirectory  (  harmonicPathPlanner          )	#moved to Processing
#add_subdirectory  (  logpolarFrameCollector       )	#moved to Visualization	
#add_subdirectory  (  logSort                      )	#moved to Visualization  
#add_subdirectory  (  microsacTest                 )	#moved to motorActions  
#add_subdirectory  (  simpleSaccade                )	#moved to motorActions  
#add_subdirectory  (  staticCalib                  )	#moved to Development
#add_subdirectory  (  objectInteractor             )	#moved to Development
#add_subdirectory  (  pythonInterface              )	#moved to Interfaces
#add_subdirectory  (  logpolar_frame_converter     )	#moved to Visualization	 
#add_subdirectory  (  logpolar_frame_converter_add )	#moved to Visualization
#add_subdirectory  (  logpolarFrameCollector	   )	#moved to Visualization
#add_subdirectory  (  testWings                    )	#moved to Development  	
#add_subdirectory  (  upSampler                    )	#moved to processing 
#add_subdirectory  (  visualAligner                )	#moved to Processing 
#add_subdirectory  (  velocityExtractor            )	#moved to Processing 
#add_subdirectory  (  zBrdInterface                )
#add_subdirectory  (  eventManager                 )  
#add_subdirectory  (  ikartDvsObstacleDetector     ) 

# --------  OPTICFLOW SUBFOLDERS ---------------------
#add_subdirectory  (  eventOpticalFlow             ) #moved to processing

# --------  TIMESTAMP-BASED OPTICAL FLOW -------------
#add_subdirectory  (  tsOptFlow                    ) #moved to processing
#add_subdirectory  (  tsOptFlowViewer              ) #moved to processing

# --------  OBJECT RECOGNITION
#add_subdirectory  ( recognitionModules/extraction ) #moved to processing
#add_subdirectory  ( recognitionModules/association) #moved to processing
#add_subdirectory  ( recognitionModules/recognition) #moved to processing
#add_subdirectory  ( recognitionModules/aggregation) #moved to processing

if(ICUB_USE_IPOPT)
#        add_subdirectory ( targetFinder                  ) #moved to processing
#	add_subdirectory (  wingsTranslator              ) #moved to processing
endif(ICUB_USE_IPOPT)

