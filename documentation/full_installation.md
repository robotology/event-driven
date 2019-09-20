# Comprehensive Installation

We'll go through the recommended set-up of `event-driven` for a first time user of the `YARP` environment. The first step is to create a directory in which to install `YARP`, `event-driven` and the eventual modules you will write. For example

> mkdir ~/yarp-install

Secondly, we want to set up some environment variables that will make the install go smoother. Use your favourite text editor to open ~/.bashrc and add the following lines:

* export INSTALL_DIR=~/yarp-install
* export YARP_DATA_DIRS=$INSTALL_DIR/share/yarp:$INSTALL_DIR/share/event-driven
* export PATH=$PATH:$INSTALL_DIR/bin

Next we want to get the required repositories. Change directory into one in which you want these projects, for example:

> mkdir ~/projects && cd ~/projects

then,

> git clone https://github.com/robotology/YCM.git

> git clone https://github.com/robotology/yarp.git

> git clone https://github.com/robotology/event-driven.git

we are going to build the repositories in the above order too. However, first you might need some extra dependencies for `YARP`. One option is to go [here](http://wiki.icub.org/wiki/Linux:Installation_from_sources) and follow the _Getting all dependencies_ instructions (either installing dependencies yourself or adding to the `apt` path).

Now you have dependencies, let's install `YCM`:

> cd ~/projects/YCM

> mkdir build && cd build

> cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

> make install -j4

then `YARP`

> cd ~/projects/yarp

> mkdir build && cd build

> cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DYCM_DIR=~/projects/YCM/build

> make install -j4

then `event-driven`

> cd ~/projects/event-driven

> mkdir build && cd build

> cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DYCM_DIR=~/projects/YCM/build -DYARP_DIR=~/projects/yarp/build

> make install -j4

[Continue with the tutorials](README.md) to test your installation, or:

### Install icub-main (optional)

`icub-main` is used if you are using the iCub robot and want to enable some of the `event-driven` modules that control the robot. To install `icub-main` do:

> cd ~/projects

> git clone https://github.com/robotology/icub-main.git

> cd icub-main

> mkdir build && cd build

> cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DYCM_DIR=~/projects/YCM/build -DYARP_DIR=~/projects/yarp/build

> make install -j4

