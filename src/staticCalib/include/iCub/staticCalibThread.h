// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file efExtractorThread.h
 * @brief Definition of a thread that receives events and extracts features
 * (see efExtractorModule.h).
 */

#ifndef _STATIC_CALIB_THREAD_H_
#define _STATIC_CALIB_THREAD_H_

#include <iostream>
#include <fstream>
#include <string>

#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Stamp.h>

#include <cv.h>
#include <highgui.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

#define LEFT    0
#define RIGHT   1

class staticCalibThread : public yarp::os::Thread {
private:

  yarp::sig::ImageOf<yarp::sig::PixelRgb> *imageL;
  yarp::sig::ImageOf<yarp::sig::PixelRgb> *imageR;
  IplImage * imgL;
  IplImage * imgR;
  
  string robotName;
  yarp::sig::Vector qL;
  yarp::sig::Vector qR;
  
  yarp::os::Semaphore* mutex;

  int numOfPairs;
  bool stereo;
  cv::Mat Kleft;
  cv::Mat Kright;
  
  cv::Mat DistL;
  cv::Mat DistR;
  double vergence;
  double version;
  

  yarp::dev::PolyDriver polyHead;
  yarp::dev::IEncoders *posHead;
  yarp::dev::IControlLimits *HctrlLim;
  
  yarp::dev::PolyDriver polyTorso;
  yarp::dev::IEncoders *posTorso;
  yarp::dev::IControlLimits *TctrlLim;

  cv::Mat R;
  cv::Mat T;
  cv::Mat Q;
  string inputLeftPortName;
  string inputRightPortName;
  string inputLeftClickName;
  string outNameRight;
  string outNameLeft;
  string camCalibFile;
  string currentPathDir;
  std::vector<string> imageListR;
  std::vector<string> imageListL;
  std::vector<std::vector<cv::Point2f> > cornerListL;
  std::vector<string> imageListLR;
  
  yarp::os::BufferedPort<yarp::os::Bottle>  imageClickInLeft;
  yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInLeft;
  
  yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInRight;
  
  yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPortRight;
  yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPortLeft;

  yarp::dev::PolyDriver* gazeCtrl;
  yarp::dev::IGazeControl* igaze;

  yarp::os::Port *commandPort;
  string imageDir;
  int startCalibration;
  int boardWidth;
  int boardHeight;
  float squareSize;
  char pathL[256];
  char pathR[256];
  void printMatrix(cv::Mat &matrix);
  bool checkTS(double TSLeft, double TSRight, double th=0.08);
  void preparePath(const char * imageDir, char* pathL, char* pathR, int num);
  void saveStereoImage(const char * imageDir, IplImage* left, IplImage * right, int num);
  void monoCalibration(const vector<string>& imageList, const std::vector<vector<cv::Point2f> > cornerList, int boardWidth, int boardHeight, cv::Mat &K, cv::Mat &Dist);
  void stereoCalibration(const vector<string>& imagelist, int boardWidth, int boardHeight,float sqsizee);
  void saveCalibration(const string& extrinsicFilePath, const string& intrinsicFilePath);
  void calcChessboardCorners(cv::Size boardSize, float squareSize, vector<cv::Point3f>& corners);
  bool updateIntrinsics( int width, int height, double fx, double fy,double cx, double cy, double k1, double k2, double p1, double p2, const string& groupname);
  bool updateExtrinsics(cv::Mat Rot, cv::Mat Tr, const string& groupname);
  void saveImage(const char * imageDir, IplImage* left, int num);
  void stereoCalibRun();
  void monoCalibRun();

public:
  staticCalibThread(yarp::os::ResourceFinder &rf, yarp::os::Port* commPort, const char *imageDir);
  void startCalib();
  void stopCalib();
  bool threadInit();
  void threadRelease();
  void run(); 
  void onStop();
  
};

#endif  //_STATIC_CALIB_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
