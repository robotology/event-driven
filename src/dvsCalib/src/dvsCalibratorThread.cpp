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
 * @file dvsCalibratorThread.cpp
 * @brief Implementation of the thread (see header dvsCalibratorThread.h)
 */

#include <iCub/dvsCalibratorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;



dvsCalibratorThread::dvsCalibratorThread() : RateThread(THRATE_CART_FRAME_INT) {
    printf("cTor\n");
    inputImageLeft          = new ImageOf<PixelMono>; 
    inputImageRight         = new ImageOf<PixelMono>;
    temporalVariation       = new ImageOf<PixelMono>;
    binsOfPoint             = new int[IMG_WIDTH*IMG_HEIGHT*2];
	traversalTable          = new bool[IMG_WIDTH*IMG_HEIGHT];    
}

dvsCalibratorThread::~dvsCalibratorThread() {
    printf("freeing memory in integrator");
      
    
}

bool dvsCalibratorThread::threadInit() {
    printf("\nInit\n");
    if (!inputPortLeft.open(getName("/cartesianImageLeft:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!inputPortRight.open(getName("/cartesianImageRight:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!outputPortLeft.open(getName("/cartesianImageLeft:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!outputPortRight.open(getName("/cartesianImageRight:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    // flags to check cameras required
    isLeftCameraActive = isRightCameraActive = false;
    count = 0;
    maxEventsInFrame = 0;
    expectedWindowSize[0]= expectedWindowSize[1]=3;
    clearingDistance=  2;
    percentOfWindowFilled = 20;
    refreshOfSummationRate = 20;
    
    
    //maxPos[0]=maxPos[1]=0;
    binCount =  0;
    totalMoment[0]=totalMoment[1]=0;
    for(int i =0; i<128; ++i){
        for(int j=0; j<128; ++j){
            binsOfPoint[i*IMG_WIDTH+2*j]=binsOfPoint[i*IMG_WIDTH+2*j+1]=-1;
        }
    }
    nbrOfBoardsVisited =0;
    // memory allocation for calibration
    image_points		    = cvCreateMat( BOARD_NBR*BOARD_WIDTH*BOARD_HEIGHT, 2, CV_32FC1 );
	object_points		    = cvCreateMat( BOARD_NBR*BOARD_WIDTH*BOARD_HEIGHT, 3, CV_32FC1 );
	point_counts			= cvCreateMat( BOARD_NBR, 1, CV_32SC1 );
	intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	distortion_coeffs	    = cvCreateMat( 5, 1, CV_32FC1 );
	//cvCloneImage(out);
    centroidImage = cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),8,1);
    out = cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),8,1);    
    calibrationDoneForRightCamera = calibrationDoneForLeftCamera = false;
	
#ifdef USE_OPEN_CV_WINDOW  
    //making a tracker-bar
    cvNamedWindow("Adjusting the blob properties");
    cvResizeWindow("Adjusting the blob properties",600,400);
    cvCreateTrackbar("height","Adjusting the blob properties", &expectedWindowSize[0], 127, NULL); // no callback
    cvCreateTrackbar("width","Adjusting the blob properties", &expectedWindowSize[1], 127, NULL); // no callback
    cvCreateTrackbar("percentage filled","Adjusting the blob properties", &percentOfWindowFilled, 100, NULL); // no callback
    cvCreateTrackbar("refresh after","Adjusting the blob properties", &refreshOfSummationRate, 200, NULL); // no callback
    cvCreateTrackbar("clearing distance","Adjusting the blob properties", &clearingDistance, 20, NULL); // no callback  
    cvNamedWindow("imageNow");
    cvNamedWindow("centroidWindow");
    cvNamedWindow("out");
#endif  
    
    return true;
       
}

void dvsCalibratorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string dvsCalibratorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void dvsCalibratorThread::resize(int width, int height) {
    printf("\nresize\n");
    this->width     = width;
    this->height    = height;
    // we can assume second camera (if active) has same dimension
    temporalVariation->resize(width,height);
    temporalVariation->zero();
    nbrOfBoardsVisited = 0; 
    
    
#ifdef USE_OPEN_CV_CORRECTION

    //Sample: load the matrices from the file
    char intrinsicParametersFile[30];
    char extrinsicParametersFile[30];
    int whichCamera = CAMERA_LEFT;
    sprintf(intrinsicParametersFile,"IntrinsicsCamera%d.xml",whichCamera);
    sprintf(extrinsicParametersFile,"DistortionCamera%d.xml",whichCamera);
    CvMat *intrinsic = (CvMat*)cvLoad(intrinsicParametersFile);
    CvMat *distortion = (CvMat*)cvLoad(extrinsicParametersFile);
    
 
    // Build the undistort map used for all subsequent frames.
 
     mapx = cvCreateImage( cvSize(128,128), IPL_DEPTH_32F, 1 );
     mapy = cvCreateImage( cvSize(128,128), IPL_DEPTH_32F, 1 );
     cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

#endif   
    resized = false;
    
}



void dvsCalibratorThread::run() {

  
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
  
  while(inputPortRight.getInputCount() && !calibrationDoneForRightCamera){          
    isRightCameraActive = true;
    inputImageRight   = inputPortRight.read(true);
    if(inputImageRight != NULL){ 
        if(!resized){
            resize(inputImageRight->width(),inputImageRight->height());
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
  
  while((!calibrationDoneForLeftCamera) && inputPortLeft.getInputCount()){          
    isLeftCameraActive = true;
    inputImageLeft   = inputPortLeft.read(true);
    if(inputImageLeft != NULL){ 
        if(!resized){
            resize(inputImageLeft->width(),inputImageLeft->height());
            resized = true;
        }
#ifdef USE_OPEN_CV_CORRECTION
        IplImage* cloneInput = cvCloneImage((IplImage*)inputImageLeft->getIplImage());
        cvRemap( cloneInput, (IplImage*)inputImageLeft->getIplImage(), mapx, mapy );
        cvNamedWindow("UndistortedImage");
        cvShowImage("UndistortedImage",(IplImage*)inputImageLeft->getIplImage());
        cvNamedWindow("OrigImage");
        cvShowImage("OrigImage",cloneInput);
        
        cvWaitKey(2);
        cvReleaseImage(&cloneInput);
        
        continue;
#endif
        
        count++;
        uchar* pimagein = (uchar*)inputImageLeft->getRawImage();
        eventsInCurrentFrame = 0;
        int padding = inputImageLeft->getPadding();
        
        int nbrOfCorners;
        if(count%refreshOfSummationRate == 0){
            findCorners(CAMERA_LEFT);  
            outputPortLeft.prepare()= *temporalVariation;
            outputPortLeft.write();
            temporalVariation->zero();
        }
        else {
            integrateCurrentFrame(inputImageLeft);
        }
     }
    

  }
  if(calibrationDoneForLeftCamera){    // free some resources for next camera in the queue
    count = 0;
    temporalVariation->zero();
  }
        
    
}
  



void dvsCalibratorThread::findCorners(int whichCamera){
 
    
    
    now = (IplImage*)temporalVariation->getIplImage();
    uchar* ptrCurrentImage = (uchar*)now->imageData;
    int counterForThreshold =0;
    int actualWidth = now->width;
    int windowWidth = 9;
    int windowHeight = 9;
    int filled = expectedWindowSize[0]*expectedWindowSize[1]*percentOfWindowFilled*.01;
    cvSet(out,cvScalar(0));
    uchar* ptrOut = (uchar*)out->imageData;
    int countOfWindows =0;
    int countForCandidatePixels[now->height][now->width];       // stores count for each bright-enough window, centered at pixel [i][j]
    for(int i=0;  i<now->height-windowHeight; ++i){
     for(int j=0; j<now->width-windowWidth; ++j){
        countForCandidatePixels[i][j] = 0;
     }
    }
    for(int i=windowHeight; i<now->height-windowHeight; ++i){
        for(int j=windowWidth; j<now->width-windowWidth; ++j){
            int valueForCurrentWindow = findBlob(now,j,i,expectedWindowSize[0],expectedWindowSize[1],thresholdForWhite);
            if( valueForCurrentWindow > filled){
                *(ptrOut+i*actualWidth+j) = 255;                             
            }
            else {
            *(ptrOut+i*actualWidth+j) = 0;
            }
         }
    }
    // centroid image stores the centroids detected.
    cvSet(centroidImage,cvScalar(0));
    uchar* ptrOutImage = (uchar*)out->imageData;
    uchar* ptrCentroid = (uchar*)centroidImage->imageData;
    
    int currentPos[2];
    currentPos[0]=currentPos[1] = 0;
    
    // set all to false
    for(int i=0; i<centroidImage->height; ++i){
        for(int j=0; j<centroidImage->width; ++j){
            traversalTable[i*IMG_WIDTH + j] = false;
        }
    }
    binCount = 0;
    for(int i=0; i<BOARD_WIDTH*BOARD_HEIGHT; ++i){
        arrayOfCentroids[2*i] = arrayOfCentroids[2*i+1] = 0;
    }
    visitAllNodes(out,centroidImage,currentPos);
    //printf("FINALLY DONE! with bins%d and boards%d\n",binCount,nbrOfBoardsVisited);
#ifdef USE_OPEN_CV_WINDOW    
    cvShowImage("centroidWindow",centroidImage);
    cvShowImage("out",out);
    cvShowImage("imageNow",now);        
    cvWaitKey(2);
#endif
    if(nbrOfBoardsVisited< BOARD_NBR){       
            prepareToCalibrate(binCount,arrayOfCentroids);
            
    }
    else if(nbrOfBoardsVisited == BOARD_NBR) {          // we are ready for calibration       
        printf("Calibration begun for camera#%d!\n",whichCamera);
        cvCalibrateCamera2( object_points, image_points, point_counts, cvGetSize( centroidImage ), 
		intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_PRINCIPAL_POINT);  // you need 2 matrices: object_point2 which has coordinate + 1 dimension for all corner points across all showings
		                                                                                // and image_point2 which has coordinate dimension for all corners across all showingCV_CALIB_FIX_ASPECT_RATIO 
	    printf("Calibration is done for camera#%d!\n",whichCamera);
	    char intrinsicParametersFile[30];
	    char extrinsicParametersFile[30];
	    char parameterFile[30];
	    sprintf(intrinsicParametersFile,"IntrinsicsCamera%d.xml",whichCamera);
	    sprintf(extrinsicParametersFile,"DistortionCamera%d.xml",whichCamera);
	    sprintf(parameterFile,"eyesDVS_%d.ini",whichCamera);
	    // Save the intrinsics and distortions
	    cvSave( intrinsicParametersFile, intrinsic_matrix );
	    cvSave( extrinsicParametersFile, distortion_coeffs );
	    FILE* paramsFilePtr;
	    paramsFilePtr = fopen(parameterFile,"w");
	    fprintf(paramsFilePtr,"[CAMERA_CALIBRATION_DVS_#%d]\n\n",whichCamera);
	    fprintf(paramsFilePtr,"projection         pinhole\ndrawCenterCross    0\nfps                30\nfpsOutputFrequency 100\n\n");
	    fprintf(paramsFilePtr,"w %d\n",IMG_WIDTH);
	    fprintf(paramsFilePtr,"h %d\n",IMG_HEIGHT);
	    fprintf(paramsFilePtr,"fx %f\n",CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ));
	    fprintf(paramsFilePtr,"fy %f\n",CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ));
	    fprintf(paramsFilePtr,"cx %f\n",CV_MAT_ELEM( *intrinsic_matrix, float, 0, 2 ));
	    fprintf(paramsFilePtr,"cy %f\n",CV_MAT_ELEM( *intrinsic_matrix, float, 1, 2 ));
	    fprintf(paramsFilePtr,"k1 %f\n",CV_MAT_ELEM( *distortion_coeffs, float, 0, 0 ));
	    fprintf(paramsFilePtr,"k2 %f\n",CV_MAT_ELEM( *distortion_coeffs, float, 1, 0 ));
	    fprintf(paramsFilePtr,"p1 %f\n",CV_MAT_ELEM( *distortion_coeffs, float, 2, 0 ));
	    fprintf(paramsFilePtr,"p2 %f\n\n",CV_MAT_ELEM( *distortion_coeffs, float, 3, 0 ));      // Not using p3!
	    
	    // Print useful info about this calibration process
	    fprintf(paramsFilePtr,"[CAMERA_CALIBRATION_DVS_#%d_CONFIGURATION]\n\n",whichCamera);
        fprintf(paramsFilePtr,"numPatternImagesRequired    \t\t\t%d\n",BOARD_NBR);
        fprintf(paramsFilePtr,"CornersX \t\t\t%d\n",BOARD_HEIGHT);
        fprintf(paramsFilePtr,"CornersY \t\t\t%d\n",BOARD_WIDTH);
        fprintf(paramsFilePtr,"patternSquareSideLength \t\t\t?\n");
        fprintf(paramsFilePtr,"outputFilename \t\t\t./eyesDVS.ini\n");
        fprintf(paramsFilePtr,"outputGroupname \t\t\tCAMERA_CALIBRATION_DVS\n");
	    fclose(paramsFilePtr);
	    
	    if(whichCamera == CAMERA_LEFT){
	        calibrationDoneForLeftCamera = true;
	    }
	    else if(whichCamera == CAMERA_RIGHT){
	        calibrationDoneForRightCamera = true;
	    }
	    else {
	        // INVALID for now
	    }
	                                                                                   
    }
    
}


void dvsCalibratorThread::prepareToCalibrate(int nbrOfCorners, int* arrayOfCorners){

    /*
    // Calibrate the camera
	cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ), 
		intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );  // you need 2 matrices: object_point2 which has coordinate + 1 dimension for all corner points across all showings
		                                                                                // and image_point2 which
		                                                                                // has coordinate dimension for all corners across all showing
    */
    // check the required nbr of points
    if(nbrOfCorners != REQUIRED_CORNERS_COUNT){
        printf("Not exact points, %d nbr of points were detected! \n",nbrOfCorners);
        return;
    }
    else{
        printf("Going to use this board!This is %d th board visited\n",nbrOfBoardsVisited);
        for(int i=0; i<REQUIRED_CORNERS_COUNT; ++i){
            printf(":%d,%d\t",*(arrayOfCorners+2*i),*(arrayOfCorners+2*i+1));
        }
        printf("\n\n");
        
        int indexNow = (nbrOfBoardsVisited)*REQUIRED_CORNERS_COUNT;                    // array stores the points of different frames, linearly	    
	    for(int i=0; i<REQUIRED_CORNERS_COUNT; ++i){
	        int j = indexNow+i;
	        CV_MAT_ELEM( *image_points, float, j, 0 ) = *(arrayOfCorners+2*i);
		    CV_MAT_ELEM( *image_points, float, j, 1 ) = *(arrayOfCorners+2*i+1);
		    CV_MAT_ELEM( *object_points, float, j, 0 ) = i/BOARD_WIDTH;                 // this assumes that the corner sent are in right order
		    CV_MAT_ELEM( *object_points, float, j, 1 ) = i%BOARD_WIDTH;
		    CV_MAT_ELEM( *object_points, float, j, 2 ) = 0.0f;                          // no Z-coordinate
	
	    }
	    CV_MAT_ELEM( *point_counts, int, nbrOfBoardsVisited, 0 ) = REQUIRED_CORNERS_COUNT;
	    nbrOfBoardsVisited += 1;
	    return;
	}
    
}

void dvsCalibratorThread::visitAllNodes(IplImage* imageGraph, IplImage* imageWithCentroidsMarked, int* posToVisit){

    uchar* rootNode = (uchar*)imageGraph->imageData;
    uchar* centroid = (uchar*)imageWithCentroidsMarked->imageData;
    
    int actualWidth = imageGraph->widthStep;
    int actualWidthCentroid = imageWithCentroidsMarked->widthStep;
    
    int jump =2;
    
    // Base case of recursion
    if(posToVisit[0]>imageGraph->height || posToVisit[1]>imageGraph->width || traversalTable[posToVisit[0]*IMG_WIDTH + posToVisit[1]]){
        return;
    }
    // Recurse!
    else{
        int tempPos[2];
        tempPos[0]=tempPos[1]=totalMoment[0]=totalMoment[1]=countOfPointsInBlob = 0;
        visitChildNode(imageGraph,posToVisit, tempPos);
        if(countOfPointsInBlob>1){
            binCount++;    
            int centroidX = totalMoment[0]/countOfPointsInBlob;
            int centroidY = totalMoment[1]/countOfPointsInBlob;
            *(centroid+actualWidthCentroid*centroidX+centroidY) = 255;
            if(binCount<=BOARD_HEIGHT*BOARD_WIDTH){
                arrayOfCentroids[2*(binCount-1)] = centroidX;
                arrayOfCentroids[2*(binCount-1)+1] = centroidY;
            }
        }
        totalMoment[0]=totalMoment[1]=countOfPointsInBlob=0;
        int nextPosInRow[2] = {max(posToVisit[0]+1,tempPos[0]+jump),posToVisit[1]};
        int nextPosInCol[2] = {posToVisit[0],max(posToVisit[1]+1,tempPos[1]+jump)};
        // flag this node as visited
        traversalTable[posToVisit[0]*IMG_WIDTH + posToVisit[1]] = true;
        
        visitAllNodes(imageGraph,imageWithCentroidsMarked,nextPosInRow);
        visitAllNodes(imageGraph,imageWithCentroidsMarked,nextPosInCol);
        return;
   }
    

    
}


void dvsCalibratorThread::visitChildNode(IplImage* imageGraph,int* presentPos, int* nextPos){

    uchar* rootNode = (uchar*)imageGraph->imageData;
    int actualWidth = imageGraph->widthStep;
    // Base case of recursion       
    if(presentPos[0]>imageGraph->height || presentPos[1]>imageGraph->width || *(rootNode+actualWidth* (presentPos[0]) + presentPos[1]) == 0
            || traversalTable[presentPos[0]*IMG_WIDTH + presentPos[1]]){
        return; // do nothing and terminate recursion
    }
    // Recurse!
    else{
        countOfPointsInBlob++;
        totalMoment[0] += presentPos[0];
        totalMoment[1] += presentPos[1];
        // store max of all X and Y positions in this region in nextPos
        if(presentPos[0] > nextPos[0]){
            nextPos[0] = presentPos[0];
        }
        if(presentPos[1] > nextPos[1]){
            nextPos[1] = presentPos[1];
        }
            
        binsOfPoint[presentPos[0]*IMG_WIDTH +2* presentPos[1]] = binCount;
        traversalTable[presentPos[0]*IMG_WIDTH + presentPos[1]] = true;
        
        int nextPosInRow[2] = {presentPos[0]+1,presentPos[1]};
        int prevPosInRow[2] = {max(0,presentPos[0]-1),presentPos[1]};
        int nextPosInCol[2] = {presentPos[0],presentPos[1]+1};
        int prevPosInCol[2] = {presentPos[0],max(0,presentPos[1]-1)};
        visitChildNode(imageGraph,nextPosInRow,nextPos);
        visitChildNode(imageGraph,prevPosInRow,nextPos);
        visitChildNode(imageGraph,nextPosInCol,nextPos);       // assuming image width 128 after padding!!
        visitChildNode(imageGraph,prevPosInCol,nextPos);
        
   }

}

int dvsCalibratorThread::findBlob(IplImage* origImgFB,int posWidth,int posHeight,  int w, int h, int threshold){

    int countOfPoints = 0;
    int imageWidth = origImgFB->widthStep;
    uchar* startingPtr = (uchar*)origImgFB->imageData;
    uchar* currentPtr = startingPtr + imageWidth*posHeight + posWidth;
    for(int i=0; i<h; ++i){
        currentPtr = startingPtr + imageWidth*(i+posHeight) + posWidth;
        for(int j=0; j<w;++j){
            if(*currentPtr > threshold){
                countOfPoints++;
            }
            currentPtr++;
        }        
    }
    return countOfPoints;    
}
 
void dvsCalibratorThread::integrateCurrentFrame(ImageOf<PixelMono>* currentFrame){

    uchar* ptrCurrentImage = currentFrame->getRawImage();
    uchar* ptrTmpVariation = temporalVariation->getRawImage();
    int padCurrentImage = currentFrame->getPadding();
    float weightageOfFrame = 1.0/(255.0);
    float wtForVal;
    float decayRate = 10.0;
    
    for(int i=0; i<currentFrame->height(); ++i){
        for(int j=0; j<currentFrame->width(); ++j){
            uchar val = *ptrCurrentImage;
          if(val > 238 && *ptrTmpVariation < 238) {
                *ptrTmpVariation = 255;
            }
            
            ptrCurrentImage++;
            ptrTmpVariation++;
        }
        ptrCurrentImage         += padCurrentImage;
        ptrTmpVariation         += padCurrentImage;
     }

    
    
}

void dvsCalibratorThread::threadRelease() {
    printf("freeing memory in integrator");
#ifdef USE_OPEN_CV_WINDOW
    cvDestroyWindow("imageNow");
    cvDestroyWindow("centroidWindow");
    cvDestroyWindow("out");
#endif
    inputPortLeft.interrupt();
    inputPortRight.interrupt();
    outputPortLeft.interrupt();
    outputPortRight.interrupt();
    inputPortLeft.close();
    inputPortRight.close();
    outputPortLeft.close();
    outputPortRight.close();
    
    delete inputImageLeft;
    delete inputImageRight;
    delete temporalVariation;
    delete [] binsOfPoint;
    delete [] traversalTable;
    cvReleaseMat(&image_points);
    cvReleaseMat(&object_points);
    cvReleaseMat(&point_counts);
    cvReleaseMat(&intrinsic_matrix);
    cvReleaseMat(&distortion_coeffs);
    cvReleaseImage(&centroidImage);
    cvReleaseImage(&out);
}





//----- end-of-file --- ( next line intentionally left blank ) ------------------

