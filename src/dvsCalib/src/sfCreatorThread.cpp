// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email:shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file sfCreatorThread.cpp
 * @brief Implementation of the thread (see header sfCreatorThread.h)
 */

#include <iCub/sfCreatorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;



sfCreatorThread::sfCreatorThread() : RateThread(THRATE_DVS_CREATOR) {
    printf("cTor\n");
    spatialFrameCreatorLeft = new sfCreator(true);
    spatialFrameCreatorRight = new sfCreator(false);
    
    leftImage = new ImageOf<PixelMono>;
    rightImage = new ImageOf<PixelMono>;
    
}

sfCreatorThread::~sfCreatorThread() {
    printf("freeing memory in integrator");
    delete spatialFrameCreatorLeft;
    delete spatialFrameCreatorRight;
    delete leftImage;
    delete rightImage;
      
    
}

bool sfCreatorThread::threadInit() {
    printf("\nInit\n");
    
    leftImage->resize(128,128);
    leftImage->zero();
    
    rightImage->resize(128,128);
    rightImage->zero();
    
    spatialFrameCreatorLeft->useCallback();
    spatialFrameCreatorLeft->open(getName("/retina/left:i").c_str());
    spatialFrameCreatorRight->useCallback();
    spatialFrameCreatorRight->open(getName("/retina/right:i").c_str());
    
    isLeftCameraActive = spatialFrameCreatorLeft->isSFCreatorInitialized();
    isRightCameraActive = spatialFrameCreatorRight->isSFCreatorInitialized();
    count = 0;    
    
    return true;
       
}

void sfCreatorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string sfCreatorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}



void sfCreatorThread::run() {

  
  // Use it!
  /*  
    if((inputPortRight.getOutputCount() || inputPortLeft.getOutputCount()) && (!isRightCameraActive|| calibrationDoneForRightCamera) && (!isLeftCameraActive|| calibrationDoneForLeftCamera)){
        IplImage* image = (IplImage*)inputImageRight->getIplImage();
	    // Example of loading these matrices back in
	    CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	    CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );

	    // Build the undistort map that we will use for all subsequent frames
	    IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	    IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	    cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

	    // Run the camera to the screen, now showing the raw and undistorted image
	    cvNamedWindow( "Undistort" ); 
	    IplImage *t = cvCloneImage( image );
		cvShowImage( "Calibration", image ); // Show raw image
		cvRemap( t, image, mapx, mapy ); // undistort image
		cvReleaseImage( &t );
		cvShowImage( "Undistort", image ); // Show corrected image
		cvWaitKey(2);
		return;

    }		
  */
  

  
  // We calibrate N (here 2) cameras serially, using same resources
/*  
  while(inputPortRight.getInputCount() && !calibrationDoneForRightCamera){          
    isRightCameraActive = true;
    inputImageRight   = inputPortRight.read(true);
    if(inputImageRight != NULL){ 
        if(!resized){
            resize(128,128);//inputImageRight->width(),inputImageRight->height());
            resized = true;
        }
        count++;
        uchar* pimagein = (uchar*)inputImageRight->getRawImage();
        eventsInCurrentFrame = 0;
        int padding = inputImageRight->getPadding();
        
        int nbrOfCorners;
        if(count%refreshOfSummationRate == 0){
            findCorners(CAMERA_RIGHT);  
            outputPortRight.prepare()= *temporalVariation;
            outputPortRight.write();
            temporalVariation->zero();
        }
        else {
            integrateCurrentFrame(inputImageRight);
        }
     }
    

  }
  if(calibrationDoneForRightCamera){    // free some resources for next camera in the queue
    count = 0;
    temporalVariation->zero();
    resized = false;
  }
*/  
    spatialFrameCreatorLeft->getMonoImage(leftImage);
    spatialFrameCreatorLeft->plotMonoImage();
    /*if(spatialFrameCreatorLeft->isSFCreatorInitialized() && (!spatialFrameCreatorLeft->getIsAccessing())){
        //spatialFrameCreatorLeft->getMonoImage(leftImage);
        spatialFrameCreatorLeft->plotMonoImage();
    }
    if(spatialFrameCreatorRight->isSFCreatorInitialized() && (!spatialFrameCreatorRight->getIsAccessing())){
        spatialFrameCreatorRight->getMonoImage(rightImage);
        spatialFrameCreatorRight->plotMonoImage();
    } */     
        
    
}

ImageOf<PixelMono >* sfCreatorThread::getTheImage(bool cam){

    if(cam)
        return leftImage;
    else
        return rightImage;
    
}

void sfCreatorThread::threadRelease() {
    printf("freeing memory in sfCreatorThread");
    delete spatialFrameCreatorLeft;
    delete spatialFrameCreatorRight;

}





//----- end-of-file --- ( next line intentionally left blank ) ------------------

