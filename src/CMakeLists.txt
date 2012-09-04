#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)

#add_subdirectory   (angel2SAC)
add_subdirectory   (aexGrabber)
add_subdirectory   (asvGrabber)
add_subdirectory   (asvBiasInterface)
add_subdirectory   (basicAexGrabber)
#add_subdirectory   (basicCartesianCollector)
add_subdirectory   (biasInterface)
#add_subdirectory   (bottleRepeater)
#add_subdirectory  (binFile2yarp)
#add_subdirectory  (buffer2image)
#add_subdirectory  (cartesian_frame_converter)
#add_subdirectory  (cartesian_frame_printer)
#add_subdirectory   (cartesianFrameCollector)
add_subdirectory  (device2yarp)
#add_subdirectory   (device2yarp2)
add_subdirectory   (dvsGrabber)
#add_subdirectory  (dvsCalibrator)
#add_subdirectory  (disparitySelector)
#add_subdirectory   (eventDriven)
#add_subdirectory   (eventSniffer) 
#add_subdirectory   (eventSelectiveAttention)
#add_subdirectory   (emorphUnmaskTest)
#add_subdirectory  (eventProcessor)
#add_subdirectory   (eventFeatureExtractor)
#add_subdirectory  (fakeEvent)
#add_subdirectory   (harmonicPathPlanner)
#add_subdirectory  (logpolarFrameCollector)
#add_subdirectory   (logSort)
#add_subdirectory   (microsacTest)
#add_subdirectory  (simpleSaccade)
add_subdirectory  (simpleSaccade)
#add_subdirectory  (objectInteractor)
#add_subdirectory   (pythonInterface)
#add_subdirectory  (logpolar_frame_converter)
#add_subdirectory  (logpolar_frame_converter_add)
#add_subdirectory   (testWings)
#add_subdirectory  (visualAligner)
add_subdirectory   (velocityExtractor)
#add_subdirectory   (eventManager)



# --------  OPTICFLOW SUBFOLDERS ---------------------
#add_subdirectory  (opticalFlow)
#add_subdirectory  (opticalFlow2)
#add_subdirectory  (opticalFlow3)
add_subdirectory   (eventOpticalFlow)

# --------  TIMESTAMP-BASED OPTICAL FLOW -------------
add_subdirectory    (tsOptFlow)

# --------  OBJECT RECOGNITION
add_subdirectory    (recognitionModules/extraction)
add_subdirectory    (recognitionModules/association)
add_subdirectory    (recognitionModules/recognition)
add_subdirectory    (recognitionModules/aggregation)
# --------  BASIC VIEWER BASED ON THE SAME UNMASK METHOD THAN TSOPTFLOW 
#   - THE PURPOSE IS ONLY TO TEST THIS METHOD
add_subdirectory    (basicCFrameConverter)

if(ICUB_USE_IPOPT)
  add_subdirectory   (targetFinder)
endif(ICUB_USE_IPOPT)

