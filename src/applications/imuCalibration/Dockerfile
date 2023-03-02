FROM ubuntu:20.04
LABEL maintainer="arren.glover@iit.it"
LABEL version="0.1"
LABEL description="Docker to install IMU calibration software"

# required libs
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends build-essential cmake libeigen3-dev freeglut3-dev gnuplot vim
RUN apt install -y git libboost-dev software-properties-common libqt5core5a
RUN apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
RUN add-apt-repository ppa:rock-core/qt4 && apt update && apt install -y libqt4-dev libqt4-opengl-dev
RUN cd /usr/include && ln -sf eigen3/Eigen Eigen && ln -sf eigen3/unsupported unsupported

# ceres-solver
RUN cd /usr/local/src && \
    git clone --depth 1 --branch 2.0.0 https://ceres-solver.googlesource.com/ceres-solver && \
    mkdir ceres-solver/build && cd ceres-solver/build && \
    cmake .. && make install -j12 

# imu_tk
RUN cd /usr/local/src && \
    git clone https://bitbucket.org/alberto_pretto/imu_tk.git 
    #&& \
    #mkdir imu_tk/build && cd imu_tk/build && \
    #cmake .. && make -j12
    
# edpr imu_tk application
RUN cd /usr/local/src && \
    git clone https://github.com/robotology/event-driven.git && \
    cd event-driven/src/applications/imuCalibration && \
    mkdir build && cd build && \
    cmake .. && make -j12

WORKDIR /usr/local/src


