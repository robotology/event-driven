#Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# After change in directory structure the uncommented modules were moved from src to /src/processing/ : 26 june 2014 by Himanshu Akolkar

cmake_minimum_required(VERSION 2.6)

add_subdirectory   (  tsOptFlow                    )
add_subdirectory   (  tsOptFlowMod                    )
add_subdirectory   (  tsOptFlowViewer              )
add_subdirectory  ( velocityExtractor    )
add_subdirectory  ( eventAnalysis    )
add_subdirectory(template)

