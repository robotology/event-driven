# Comprehensive Installation

These installation instructions are for first time users of YARP and specifically for compiling from sources in a linux environment. The instructions for installing YARP can be found [here](http://wiki.icub.org/wiki/Linux:Installation_from_sources) and are repeated here for completeness. For any problems please check the official instructions.

## Install YARP

set up the required environment variables. 

We will assume that you are installing YARP in the directory $PROJECTS - for example this could be your home directory. Use your favourite text editor to open ~/.bashrc and add the following lines:

* export YARP_ROOT=$PROJECTS/yarp
* export YARP_DIR=$YARP_ROOT/build
* export ICUBcontrib_DIR=$PROJECTS/icub-contrib-install
* export YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUBcontrib_DIR/share/ICUBcontrib
* export PATH=$PATH:$YARP_DIR/bin:$ICUBcontrib_DIR/bin

replace $PROJECTS with your desired directory.

get all dependencies

> sudo sh -c 'echo "deb http://www.icub.org/ubuntu xenial contrib/science" > /etc/apt/sources.list.d/icub.list'

> sudo apt update 

> sudo apt install icub-common

get the repositories

> git clone https://github.com/robotology/yarp.git

> git clone https://github.com/robotology/icub-contrib-common.git

> git clone https://github.com/robotology/event-driven.git

build the repositories

> cd $YARP_ROOT

> mkdir build && cd build

> ccmake ..

set the following variables, pressing [c] to configure until all options are shown:

* CREATE_GUIS ON
* CREATE_LIB_MATH ON
* CREATE_YARPDATADUMPER ON
* CREATE_YARPDATAPLAYER ON
* CREATE_YARPLOGGER ON
* CREATE_YARPMANAGER ON
* CREATE_YARPSCOPE ON
* CREATE_YARPVIEW ON

if you are using the icub robot also turn on:

* CREATE_YARPMOTORGUI ON
* CREATE_YARPROBOTINTERFACE ON

finally press [g] to generate the makefile.

> make -j4

> cd ../..

> icub-contrib-common

> mkdir build && cd build

> ccmake ..

set the following variables

* CMAKE_INSTALL_DIR=$ICUBcontrib_DIR

press [c] to configure and [g] to generate

> make install -j4

> cd ../..

> cd event-driven

> mkdir build && cd build

> ccmake ..

set the following variables, pressing [c] to configure until all options are shown:

* BUILD_PROCESSING ON
* BUILD_APPLICATIONS ON

select any of the applications you want to build by setting them to ON. If you need to interface to hardware (e.g. using the zynqGrabber), please also set:

* BUILD_PROCESSING ON

and select which modules you need. In addition several options need to be set in regards to the hardware parameters, which will also need to be correct for pre-recorded sequences.

> VLIB_CLOCK_PERIOD_NS 80

> VLIB_CODEC_128x128 OFF

> VLIB_TIMER_BITS 24

These are customisable for your hardware, specifying the event timing and the decoding method, which is different for the 128x128 resolution DVS and a higher resolution camera.

Press [g] to generate the makefile.

> make install -j4

completed.

## Install icub-main (optional)

This is only needed if you are going to work with an icub robot and is not needed if you want to use the event-driven library as a stand-alone project.

Add the following environment variables (e.g. in ~/.bashrc):

* export ICUB_ROOT=$PROJECTS/icub-main
* export ICUB_DIR=$ICUB_ROOT/build
* export YARP_DATA_DIRS=$YARP_DATA_DIRS:$ICUB_DIR/share/icub
* export PATH=$PATH:$ICUB_DIR/bin

in the $PROJECTS directory

> git clone https://github.com/robotology/icub-main.git

> cd icub-main

> mkdir build && cd build

> ccmake ..

set the following variables, pressing [c] to configure until all options are shown:

* ENABLE_icubmod_cartesiancontrollerclient ON
* ENABLE_icubmod_cartesiancontrollerserver ON
* ENABLE_icubmod_gazecontrollerclient ON

finally press [g] to generate.

> make -j4

completed

## Test Installation

To test your installation is correct please follow the "Getting started with the viewer" tutorial [here](http://robotology.github.io/event-driven/doxygen/doc/html/pages.html).

