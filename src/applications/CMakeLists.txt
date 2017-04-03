cmake_minimum_required(VERSION 2.6)

option(ENABLE_autosaccade "Build autosaccade module" OFF)
option(ENABLE_gazedemo "Build ball gaze following demo" OFF)
option(ENABLE_reptest "Build representation tester" OFF)
option(ENABLE_vergence "Build vergence control demo" OFF)

if(ENABLE_autosaccade)
    add_subdirectory(autosaccade)
endif(ENABLE_autosaccade)

if(ENABLE_gazedemo)
    add_subdirectory(gazeDemo)
endif(ENABLE_gazedemo)

if(ENABLE_reptest)
    add_subdirectory(reptest)
endif(ENABLE_reptest)

if(ENABLE_vergence)
    add_subdirectory(vergenceDemo)
endif(ENABLE_vergence)
