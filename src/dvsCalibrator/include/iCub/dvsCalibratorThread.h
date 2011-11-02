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
 * @file dvsCalibrator.h
 * @brief Definition of a thread that takes the cartesian images from the cartesian frame collector, stabilizes them by suitable summation over many frames, extracts the known points of a (spatially
 * distributed) grid of points through centroid detection and finally uses these points for camera calibration. In case of more than one camera, this procedure is followed sequentially.
 */

#ifndef _CARTESIAN_FRAME_INTEGRATOR_THREAD_H_
#define _CARTESIAN_FRAME_INTEGRATOR_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>

//#include <yarp/sig/IplImage.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>



//#include <iCub/logPolar.h>




//#include <Eigen/Dense>


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define MONO_PIXEL_SIZE 1
#define THRATE_CART_FRAME_INT 25

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif

//some config values
#define BOARD_NBR 8
#define BOARD_WIDTH 8
#define BOARD_HEIGHT 8
#define REQUIRED_CORNERS_COUNT 64 
#define IMG_WIDTH 128
#define IMG_HEIGHT 128
#define NO_STEREO
#define CAMERA_LEFT 0
#define CAMERA_RIGHT 1


class dvsCalibratorThread : public yarp::os::RateThread{ 

private:
    
    std::string name;                                                                       // rootname of all the ports opened by this thread
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inputPortLeft;        //input port for images from the left camera 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inputPortRight;       //input port for images from the right camera 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputPortLeft;       //output port for images from the left camera 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputPortRight;      //output port for images from the right camera   
    
    yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImageLeft;                               // instantaneous image from left camera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImageRight;                              // instantaneous image from right camera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* temporalVariation;                            // for each pixel, this stores the time since last firing
    
    int width;                                                                              // width of the image
    int height;                                                                             // height of the image 
    int count;                                                                              // counters to keep track of frames received
    int frameCount;                                                                         // number of frames that are assimilated into one   
    
    bool resized;
    bool isLeftCameraActive;
    bool isRightCameraActive;
    
    IplImage* chess, *blank;    // To be REMOVED
    int maxEventsInFrame;
    int eventsInCurrentFrame;
    
    int expectedWindowSize[2];                                                              // expected size of the blob in [ht][wd] format
    int percentOfWindowFilled;                                                              // percent (in 100) of window area that is expected to be filled
    int thresholdForWhite;                                                                  // threshold above which pixel is considered to be white // not needed though
    int refreshOfSummationRate;                                                             // summation of images (till the count reaches this) for stabilization
    int clearingDistance;                                                                   // distance within which two points are considered similar
    int* binsOfPoint;//[128][128][2];                                                       // bin number and centroid of bin for any given pixel
    int arrayOfCentroids[2*BOARD_WIDTH*BOARD_HEIGHT];
    int nbrOfBoardsVisited;
    int binCount;
    int countOfPointsInBlob;
    long int totalMoment[2];                                                                // moment of a connected region in X and Y direction, useful for centroid calculation
    bool* traversalTable;//[128][128];
    
    
    CvMat* image_points	;                                                                   // Memory allocation for calibration method
	CvMat* object_points;
	CvMat* point_counts;
	CvMat* intrinsic_matrix;
	CvMat* distortion_coeffs;
	IplImage* centroidImage;
	int countOfBoardNbr;
	bool calibrationDoneForRightCamera,calibrationDoneForLeftCamera;
    

public:
    /**
    * constructor
    */
    dvsCalibratorThread();

    /**
     * destructor
     */
    ~dvsCalibratorThread();

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
    
    /**
    * function that stabilizes images
    * @param currentFrame the input image
    */
    void integrateCurrentFrame(yarp::sig::ImageOf<yarp::sig::PixelMono>* currentFrame);
    
    /**
    * function to find corners in an image
    * @param whichCamera int to enumerate the camera LEFT:0 RIGHT:1
    */
    void findCorners(int whichCamera);
    
    /**
    * Given a point in an image, and a fixed window size, counts the points within this window that have values above the threshold
    * @param img the input image
    * @param posWidth ordinate of the point in image
    * @param posHeight abcissa of the point in image
    * @param w width of the window
    * @param h height of the window
    * @param threshold value above which point in the vicinity is counted    
    */
    int findBlob(IplImage* img,int posWidth,int posHeight,  int w, int h, int threshold);
    
    /**
    * This performs simple search of nearby bright points around a point via recursion
    * @param rootNode the starting point of the graph, here 1st pixel address
    * @param presentPos the node visited now
    * @param nextPos the node to be visited next
    */
    void visitChildNode(IplImage* rootNode,int* presentPos, int* nextPos);
    
    /**
    * This performs node search (visitChildNode) on all unvisited parts of an image
    * @param rootNode the starting point of the graph, here 1st pixel address
    * @param centroidImage openCV image to visualize the centroids detected
    * @param arrayOfCorners array of coordinates of centroids detected stored in [x1,y1],[x2,y2].. fashion
    */
    void visitAllNodes(IplImage* rootNode, IplImage* centroidImage, int* posToVisit);
    
    /**
    * Given array of coordinates of the points, this populates corresponding openCV matrices, if the points match exactly.
    * @param nbrOfCorners number of points detected, they should exactly match expected number of points
    * @param array of coordinates of centroids detected stored in [x1,y1],[x2,y2].. fashion
    */
    void prepareToCalibrate(int nbrOfCorners, int* arrayOfCorners);


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

    
    
    
    
};




#endif  //_CARTESIAN_FRAME_INTEGRATOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

