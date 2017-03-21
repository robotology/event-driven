# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Arren Glover
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME vVergence)
project(${PROJECTNAME})

add_subdirectory(depthGT)
add_subdirectory(vergenceController)
