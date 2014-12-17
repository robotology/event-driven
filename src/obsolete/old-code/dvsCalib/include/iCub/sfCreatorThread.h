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
 * @file sfCreatorThread.h
 * @brief Definition of a thread that takes care of handling in coming events and passes it on for construction of spatial frame
 */

#ifndef _SPATIAL_FRAME_CREATOR_THREAD_H_
#define _SPATIAL_FRAME_CREATOR_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>

//#include <yarp/sig/IplImage.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>



//#include <iCub/logPolar.h>
#include <iCub/sfCreator.h>



//#include <Eigen/Dense>


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

# define THRATE_DVS_CREATOR 15

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif



class sfCreatorThread : public yarp::os::RateThread{ 

private:
    
    std::string name;                                                                       // rootname of all the ports opened by this thread
    
       
    yarp::sig::ImageOf<yarp::sig::PixelMono>* leftImage;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* rightImage;
   
    int width;                                                                              // width of the image
    int height;                                                                             // height of the image 
    int count;                                                                              // counters to keep track of frames received
    int frameCount;                                                                         // number of frames that are assimilated into one   
    
    bool resized;
    bool isLeftCameraActive;
    bool isRightCameraActive;
    
    
	
	
    

public:
    /**
    * constructor
    */
    sfCreatorThread();

    /**
     * destructor
     */
    ~sfCreatorThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that resizes the necessary and already allocated images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resize(int width, int height);    


     /* to suspend and resume the thread processing
    */
    void suspend(){
        printf("suspending edges thread\n");
        RateThread::suspend();      // LATER: some sanity checks
    }

    void resume(){
        printf("resuming edges thread\n");
        RateThread::resume();
    }
    
    yarp::sig::ImageOf<yarp::sig::PixelMono>* getTheImage(bool cam=true);
    
    
    sfCreator* spatialFrameCreatorLeft;
	sfCreator* spatialFrameCreatorRight;

    
    
    
    
};




#endif  //_SPATIAL_FRAME_CREATOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

