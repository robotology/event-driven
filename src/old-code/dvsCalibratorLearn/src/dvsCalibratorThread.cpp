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
#include <iCub/fittingModel.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>

int clickX, clickY, countOfPoints,countOfBoards;
bool downFirst, rightDown,renew, isFirstTime;
const char* nChoosingWindow = "Choose points";
// Implement mouse callback
void my_mouse_callback( int event, int x, int y, int flags, void* param ){
	IplImage* image = (IplImage*) param;

	switch( event ){
		case CV_EVENT_MOUSEMOVE:
			// nothing
			break;

		case CV_EVENT_LBUTTONDOWN:
			downFirst = true;
			break;

		case CV_EVENT_LBUTTONUP:
			if(downFirst)
			{

			    clickX = x;
			    clickY = y;
			    downFirst = false;
			    renew = false;
			    countOfPoints++;
			    if(countOfPoints>=BOARD_HEIGHT*BOARD_WIDTH)
			    {
			        countOfPoints = BOARD_HEIGHT*BOARD_WIDTH-1;
			    }
			}
			break;
        case CV_EVENT_RBUTTONDOWN:
			rightDown = true;
			break;

		case CV_EVENT_RBUTTONUP:
			if(rightDown)
			{
			    renew = true;
			    rightDown = false;
			    downFirst = false;
			    countOfBoards++;
			    if(countOfBoards>=BOARD_NBR)
			    {
			        countOfBoards = BOARD_NBR;
			    }
			    countOfPoints = -1;
			}
			break;

	}
}

// Returns a new image that is a cropped version (rectangular cut-out)
// of the original image.
IplImage* cropImage(const IplImage *img, const CvRect region)
{
	IplImage *imageCropped;
	CvSize size;

	if (img->width <= 0 || img->height <= 0
		|| region.width <= 0 || region.height <= 0) {
		//cerr << "ERROR in cropImage(): invalid dimensions." << endl;
		exit(1);
	}

	if (img->depth != IPL_DEPTH_8U) {
		//cerr << "ERROR in cropImage(): image depth is not 8." << endl;
		exit(1);
	}

	// Set the desired region of interest.
	cvSetImageROI((IplImage*)img, region);
	// Copy region of interest into a new iplImage and return it.
	size.width = region.width;
	size.height = region.height;
	imageCropped = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageCropped);	// Copy just the region.

	return imageCropped;
}

// Creates a new image copy that is of a desired size. The aspect ratio will
// be kept constant if 'keepAspectRatio' is true, by cropping undesired parts
// so that only pixels of the original image are shown, instead of adding
// extra blank space.
// Remember to free the new image later.
IplImage* resizeImage(const IplImage *origImg, int newWidth,
	int newHeight, bool keepAspectRatio)
{
	IplImage *outImg = 0;
	int origWidth;
	int origHeight;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0
		|| origWidth <= 0 || origHeight <= 0) {
		//cerr << "ERROR: Bad desired image size of " << newWidth
		//	<< "x" << newHeight << " in resizeImage().\n";
		exit(1);
	}

	if (keepAspectRatio) {
		// Resize the image without changing its aspect ratio,
		// by cropping off the edges and enlarging the middle section.
		CvRect r;
		// input aspect ratio
		float origAspect = (origWidth / (float)origHeight);
		// output aspect ratio
		float newAspect = (newWidth / (float)newHeight);
		// crop width to be origHeight * newAspect
		if (origAspect > newAspect) {
			int tw = (origHeight * newWidth) / newHeight;
			r = cvRect((origWidth - tw)/2, 0, tw, origHeight);
		}
		else {	// crop height to be origWidth / newAspect
			int th = (origWidth * newHeight) / newWidth;
			r = cvRect(0, (origHeight - th)/2, origWidth, th);
		}
		IplImage *croppedImg = cropImage(origImg, r);

		// Call this function again, with the new aspect ratio image.
		// Will do a scaled image resize with the correct aspect ratio.
		outImg = resizeImage(croppedImg, newWidth, newHeight, false);
		cvReleaseImage( &croppedImg );

	}
	else {

		// Scale the image to the new dimensions,
		// even if the aspect ratio will be changed.
		outImg = cvCreateImage(cvSize(newWidth, newHeight),
			origImg->depth, origImg->nChannels);
		if (newWidth > origImg->width && newHeight > origImg->height) {
			// Make the image larger
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_LINEAR: good at enlarging.
			// CV_INTER_CUBIC: good at enlarging.
			cvResize(origImg, outImg, CV_INTER_LINEAR);
		}
		else {
			// Make the image smaller
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_AREA: good at shrinking (decimation) only.
			cvResize(origImg, outImg, CV_INTER_AREA);
		}

	}
	return outImg;
}

void print_state (size_t iter, gsl_multifit_fdfsolver * s)
{
    printf ("iter: %3d x = %f %f %f %f Norm = %f\n",
            (int)iter,
            gsl_vector_get (s->x, 0),
            gsl_vector_get (s->x, 1),
            gsl_vector_get (s->x, 2),
            //gsl_vector_get (s->x, 3),
            gsl_blas_dnrm2 (s->f));
}


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;



dvsCalibratorThread::dvsCalibratorThread() : RateThread(THRATE_DVS_CALIB)
{
    printf("cTor\n");
    inputImageLeft          = new ImageOf<PixelMono>;
    inputImageRight         = new ImageOf<PixelMono>;
    tempVariation           = new ImageOf<PixelMono>;
    binsOfPoint             = new int[IMG_WIDTH*IMG_HEIGHT*2];
    traversalTable          = new bool[IMG_WIDTH*IMG_HEIGHT];

}

dvsCalibratorThread::~dvsCalibratorThread()
{
    printf("freeing memory in integrator");


}

bool dvsCalibratorThread::threadInit()
{
    printf("\nInit\n");
    /*
    if (!inputPortLeft.open(getName("/cartesianImageLeft:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!inputPortRight.open(getName("/cartesianImageRight:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    */
    if (!outputPortLeft.open(getName("/cartesianImageLeft:o").c_str()))
    {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPortRight.open(getName("/cartesianImageRight:o").c_str()))
    {
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
    distortionParameter[0]=distortionParameter[1]=500;
    sensitivityOfDistort[0]=6;
    sensitivityOfDistort[1]=6;
    sensitivityOfFocalLength[0]=0;
    sensitivityOfFocalLength[1] = 0;
    focalLengths[0]=focalLengths[1]=510;
    imageCenter[0]=imageCenter[1]=64;
    saveOpt = 0;


    // Hough related
    houghMethod = 2;
    houghDThetaVal = 1000.0*CV_PI/180 ;
    houghDThetaScale = 8;
    houghDRhoVal = 4;
    houghDRhoScale = 4;
    houghThreshold = 40;
    minLineLength = 30;
    maxGapLength = 10;
    countForIntImg = 50;


    //maxPos[0]=maxPos[1]=0;
    binCount =  0;
    totalMoment[0]=totalMoment[1]=0;
    for(int i =0; i<128; ++i)
    {
        for(int j=0; j<128; ++j)
        {
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

    cvNamedWindow("Adjusting the distortion");
    cvResizeWindow("Adjusting the distortion",600,400);
    cvCreateTrackbar("K1","Adjusting the distortion", &distortionParameter[0], 1000, NULL);
    cvCreateTrackbar("factor for K1","Adjusting the distortion", &sensitivityOfDistort[0], 10, NULL);
    cvCreateTrackbar("K2","Adjusting the distortion", &distortionParameter[1], 1000, NULL);
    cvCreateTrackbar("factor for K2","Adjusting the distortion", &sensitivityOfDistort[1], 10, NULL);
    cvCreateTrackbar("Fx","Adjusting the distortion", &focalLengths[0], 1000, NULL);
    cvCreateTrackbar("factor for Fx","Adjusting the distortion", &sensitivityOfFocalLength[0], 10, NULL);
    cvCreateTrackbar("Fy","Adjusting the distortion", &focalLengths[1], 1000, NULL);
    cvCreateTrackbar("factor for Fy","Adjusting the distortion", &sensitivityOfFocalLength[1], 10, NULL);
    cvCreateTrackbar("Cx: Image Center","Adjusting the distortion", &imageCenter[0], 128, NULL);
    cvCreateTrackbar("Cy: Image Center","Adjusting the distortion", &imageCenter[1], 128, NULL);


    cvCreateTrackbar("Save?(none/K1/K2/all)","Adjusting the distortion", &saveOpt, 3, NULL);


    cvNamedWindow("Hough parameters");
    cvResizeWindow("Hough parameters",600,600);
    cvCreateTrackbar("Method","Hough parameters",&houghMethod,3,NULL);
    cvCreateTrackbar("dTheta Value","Hough parameters",&houghDThetaVal,100,NULL);
    cvCreateTrackbar("dTheta Scale","Hough parameters",&houghDThetaScale,10,NULL);
    cvCreateTrackbar("dRho Val","Hough parameters",&houghDRhoVal,100,NULL);
    cvCreateTrackbar("dRhoScale","Hough parameters",&houghDRhoScale,10,NULL);
    cvCreateTrackbar("Threshold","Hough parameters",&houghThreshold,100,NULL);
    cvCreateTrackbar("MinLength","Hough parameters",&minLineLength,100,NULL);
    cvCreateTrackbar("Max Gap","Hough parameters",&maxGapLength,100,NULL);
    cvCreateTrackbar("Count For Integral","Hough parameters",&countForIntImg,200,NULL);


    cvNamedWindow(nChoosingWindow);
    choosePoints = cvCreateImage(cvSize(128,128),8,1);
    // Set up the callback
	cvSetMouseCallback( nChoosingWindow, my_mouse_callback, NULL);



    distortParams = fopen("DistortionParameters.txt","w");


    inputImageLeft->resize(128,128);
    inputImageLeft->zero();
    tempVariation->resize(128,128);
    tempVariation->zero();
    spatialFrameCreatorThread = new sfCreatorThread();
    spatialFrameCreatorThread->start();

    clickX= clickY= countOfPoints=0;
    for(int i=0; i<BOARD_HEIGHT*BOARD_WIDTH;++i)
    {
        for(int j=0; j<BOARD_NBR; ++j)
        {
            chosenPoints[j][i].x=0;
            chosenPoints[j][i].y=0;
        }

    }

    /*const int pts = LM_NBR_OF_POINTS;
    double dataPts[pts];
    double sigmaPts[pts];
    // This is the data to be fitted /
    gsl_rng* randgen = gsl_rng_alloc(gsl_rng_default);
       for (int i = 0; i < pts; i++)
         {
           double t = i;
           double ran = gsl_ran_gaussian (randgen, 0.1);
           dataPts[i] = 1.0 + 5 * exp (-0.1 * t)
                      + ran;
           sigmaPts[i] = 0.1;
           printf ("data: %u %g %g %f \n", i, dataPts[i], sigmaPts[i],ran);
         };
    LMCurveFitter           = new LMCurveFit(3,pts,dataPts,sigmaPts);
    */
    return true;

}

void dvsCalibratorThread::setName(string str)
{
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string dvsCalibratorThread::getName(const char* p)
{
    string str(name);
    str.append(p);
    return str;
}

void dvsCalibratorThread::resize(int width, int height)
{
    this->width     = width;
    this->height    = height;
    // we can assume second camera (if active) has same dimension
    printf("resizing tempVariation for dvsCalib\n");
    //tempVariation->resize(128,128);
    //tempVariation->zero();
    printf("Done with temporal varaition\n");
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
    resized = true;
    cout<<"Going to rectify images\n";
    //getRectifiedImages();
    printf("\nresized\n");

}

void dvsCalibratorThread::undistort(IplImage* imageTobeUndistorted, IplImage* retImage, double K1, double K2)
{

    //IplImage* straightLine = cvCreateImage(cvSize(128,128),8,1);
    //cvSet(straightLine,cvScalar(0));
    uchar* ptrStraightLine = (uchar*)imageTobeUndistorted->imageData;
    uchar* ptrRetImage = (uchar*)retImage->imageData;
    int widthStarightLine = imageTobeUndistorted->widthStep;

    for(int i=0; i< imageTobeUndistorted->height; ++i)
    {
        for(int j=0; j< imageTobeUndistorted->width; ++j)
        {
            float yVal = (float)(64-i);
            float xVal = (float)(j-64);
            double t = atan2(yVal,xVal);
            double r = sqrt(yVal*yVal + xVal*xVal)/64.0;

            // apply distortion
            r = 64.0*r*(1.0 + K1* r * r + K2 * r * r * r * r);
            int newWid = 64 + (int)(r*cos(t));
            int newHgt = 64 - (int)(r*sin(t));
            if(newWid>=0 && newWid<=127 && newHgt>=0 && newHgt<=127)
            {
                *(ptrRetImage + widthStarightLine*newHgt + newWid) = *(ptrStraightLine + widthStarightLine*i + j);
            }

        }
    }


}

void dvsCalibratorThread::getRectifiedImages()
{
     // Integrate the 500 images
    IplImage* integratedImage = cvCreateImage(cvSize(128,128),32,1);
    IplImage* integratedColorImage = cvCreateImage(cvSize(128,128),32,3);

    float* ptrIntImage = (float*)integratedImage->imageData;
    int wdt = integratedImage->widthStep /sizeof(float);

    const int _poses = 6;
    int indexOfFolders[_poses]={1,3,5,6,7,9};
    int in=0;
    int intFrameCount = 100;
    while(in<_poses)
    {
        for(int ind=0; ind<500-intFrameCount; ++ind)
        {

            cvSet(integratedImage,cvScalar(0));
            char nameImg[20];
            sprintf(nameImg,"./dump_0000%d/0000000%d.ppm",indexOfFolders[in],ind);
            if(ind<10)
            {
                sprintf(nameImg,"./dump_0000%d/0000000%d.ppm",indexOfFolders[in],ind);
            }
            else if(ind<100)
            {
                sprintf(nameImg,"./dump_0000%d/000000%d.ppm",indexOfFolders[in],ind);
            }
            else
                sprintf(nameImg,"./dump_0000%d/00000%d.ppm",indexOfFolders[in],ind);
            IplImage* curImg = cvLoadImage(nameImg,CV_LOAD_IMAGE_GRAYSCALE);
            if (curImg == NULL) continue;
            uchar* ptrCurImg = (uchar*)curImg->imageData;
            int width = curImg->widthStep;
            for(int innerInd=ind+1; innerInd<ind+intFrameCount; ++innerInd)
            {
                char nameImg2[20];
                if(innerInd<10)
                {
                    sprintf(nameImg2,"./dump_0000%d/0000000%d.ppm",indexOfFolders[in],innerInd);
                }
                else if(innerInd<100)
                {
                    sprintf(nameImg2,"./dump_0000%d/000000%d.ppm",indexOfFolders[in],innerInd);
                }
                else
                    sprintf(nameImg2,"./dump_0000%d/00000%d.ppm",indexOfFolders[in],innerInd);
                IplImage* nxtImg = cvLoadImage(nameImg2,CV_LOAD_IMAGE_GRAYSCALE);
                if( nxtImg == NULL) break;

                uchar* ptrNxtImg = (uchar*)nxtImg->imageData;
                for(int ht=0; ht<128; ++ht)
                {
                    for(int wd=0; wd<128; ++wd)
                    {
                        *(ptrIntImage + ht*wdt + wd) = *(ptrIntImage + ht*wdt + wd) + (float)(*(ptrNxtImg + ht*width + wd))/(50.0*64.0);
                    }
                }
                cvReleaseImage(&nxtImg);
            }
            int countInt = 0;
            for(int i=0; i<128; ++i)
            {
                for(int j=0; j<128; ++j)
                {
                    if(*(ptrIntImage + i*wdt + j) > .2) countInt++;
                    /*if(*(ptrIntImage + i*wdt + j) < 10){
                        *(ptrIntImage + i*wdt + j) = 0;
                    }
                    else{
                        *(ptrIntImage + i*wdt + j)= 255;
                    }*/
                }
            }
            cvReleaseImage(&curImg);
            if(countInt>countForIntImg)
            {
                //cvNamedWindow("Integrated Image");
                //cvShowImage("Integrated Image",integratedImage);
                //if(cvWaitKey(0) == 'y')
                break;
                //cvWaitKey(0);
            }
            //else printf("Count is%d\n", countInt);

        }

        IplImage* grayScale = cvCreateImage(cvGetSize(integratedImage),8,1);
        cvConvertScale( integratedImage , grayScale, 255, 0);
        char nameImgRect[20];
        sprintf(nameImgRect,"./rectified/R%d.ppm",in);
        cvSaveImage(nameImgRect,grayScale);
        cout<<"Saved in folder"<<indexOfFolders[in]<<endl;

        cvReleaseImage(&grayScale);


        in++;
    }
    cvReleaseImage(&integratedImage);
    cvReleaseImage(&integratedColorImage);

    //Choose corners
    //For each board, for each corner, store X then Y image coordinate




    countOfBoards = countOfPoints = -1;
    renew = false;
    int totalCorners =0;
    char nameImgRect[20];
    sprintf(nameImgRect,"./rectified/R%d.ppm",indexOfFolders[0]);
    IplImage* _tmp1 = cvLoadImage(nameImgRect,CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *magIm = cvCreateImage(cvSize(128*MAG_FACT,128*MAG_FACT),8,1);
    cvResize(_tmp1,magIm,CV_INTER_CUBIC);
    //IplImage *magIm = resizeImage(_tmp1, 128*MAG_FACT,128*MAG_FACT, true);
    IplImage* boardColor = cvCreateImage(cvSize(128*MAG_FACT,128*MAG_FACT),8,3);
    cvCvtColor(magIm, boardColor, CV_GRAY2BGR );
    cvReleaseImage(&_tmp1);
    //cvReleaseImage(&magIm);

    CvPoint shownPoints[BOARD_NBR][BOARD_HEIGHT*BOARD_WIDTH];

/*
    do
    {
        if(renew)
        {
            sprintf(nameImgRect,"./rectified/R%d.ppm",countOfBoardNbr%_poses);
            IplImage* _tmp = cvLoadImage(nameImgRect,CV_LOAD_IMAGE_GRAYSCALE);
            //IplImage *magIm = resizeImage(_tmp1, 128*MAG_FACT,128*MAG_FACT, true);
            cvResize(_tmp,magIm,CV_INTER_CUBIC);

            cvCvtColor(magIm, boardColor, CV_GRAY2BGR );
            cvReleaseImage(&_tmp);
            //cvReleaseImage(&magIm);
            cout<<"Board#"<<countOfBoards<<"  Corner Nbr#"<<countOfPoints<<endl;
            renew = false;
            for(int i=0; i<BOARD_NBR;++i)
            {
                for(int j=0; j<BOARD_HEIGHT*BOARD_WIDTH; ++j)
                {
                    cout<<chosenPoints[i][j].x<<"  "<<chosenPoints[i][j].y<<"  ";
                }
                cout<<endl;
            }
        }
        cvShowImage(nChoosingWindow,boardColor);
        cvWaitKey(1);
        if(countOfBoards >=0 && countOfPoints >=0 && (clickX!=0 || clickY !=0))
        {
            countOfBoardNbr=countOfBoards;
            (chosenPoints[countOfBoards][countOfPoints]).x=clickX%MAG_FACT<MAG_FACT/2? clickX/MAG_FACT: clickX/MAG_FACT + 1;
            (chosenPoints[countOfBoards][countOfPoints]).y=clickY%MAG_FACT<MAG_FACT/2? clickY/MAG_FACT: clickY/MAG_FACT + 1;

            (shownPoints[countOfBoards][countOfPoints]).x=clickX;
            (shownPoints[countOfBoards][countOfPoints]).y=clickY;

            int indexNow = (countOfBoards)*REQUIRED_CORNERS_COUNT;
            int j = indexNow+countOfPoints;
            CV_MAT_ELEM( *image_points, float, j, 0 ) = clickX;
            CV_MAT_ELEM( *image_points, float, j, 1 ) = clickY;
            CV_MAT_ELEM( *object_points, float, j, 0 ) = countOfPoints/BOARD_WIDTH;                 // this assumes that the corner sent are in right order
            CV_MAT_ELEM( *object_points, float, j, 1 ) = countOfPoints%BOARD_WIDTH;
            CV_MAT_ELEM( *object_points, float, j, 2 ) = 0.0f;                          // no Z-coordinate
            cvCircle( boardColor, (shownPoints[countOfBoards][countOfPoints]),1,CV_RGB(0,255,0));
            totalCorners++;
        }



    }while(countOfBoards<BOARD_NBR);

    // Done with prep
    FILE* pointInFile;
    pointInFile = fopen("calibrating_points","w");
    cout<<"DONE!"<<endl;
    for(int i=0; i<BOARD_NBR; ++i)
    {
        for(int j=0; j<BOARD_HEIGHT; ++j)
        {
            for(int k=0; k<BOARD_WIDTH; ++k)
                fprintf(pointInFile,"%d %d \t",chosenPoints[i][j*BOARD_WIDTH +k].x,chosenPoints[i][j*BOARD_WIDTH +k].y);
            fprintf(pointInFile,"\n");
        }

    }

*/
    // Rewrite points
    int ptArray[BOARD_NBR*BOARD_HEIGHT*BOARD_WIDTH*2] =
    {
        28, 32, 43, 31, 59, 32, 76, 33, 92, 34, 26, 48, 41, 48, 58, 49, 75, 49, 93, 50, 25, 65, 41, 66, 57, 67, 75, 68, 92, 69, 25, 82, 40, 84, 56, 86, 74, 86, 90, 87,
29, 32, 43, 31, 59, 31, 76, 32, 92, 34, 26, 48, 42, 48, 58, 48, 76, 49, 93, 51, 25, 65, 40, 66, 57, 67, 75, 68, 92, 69, 25, 82, 39, 83, 56, 86, 73, 87, 90, 87,
35, 33, 47, 34, 59, 35, 73, 38, 87, 40, 32, 45, 43, 47, 56, 49, 69, 52, 84, 56, 28, 58, 40, 61, 52, 65, 65, 68, 80, 72, 26, 72, 37, 75, 48, 79, 62, 83, 76, 87,
20, 30, 32, 28, 46, 26, 60, 26, 75, 26, 20, 42, 32, 42, 46, 41, 60, 40, 75, 40, 19, 57, 32, 57, 46, 56, 60, 56, 76, 56, 20, 72, 32, 72, 46, 72, 61, 72, 76, 72,
28, 32, 43, 31, 59, 31, 77, 32, 92, 33, 27, 48, 42, 48, 59, 49, 75, 49, 92, 51, 25, 65, 41, 66, 57, 67, 75, 68, 92, 69, 25, 82, 40, 84, 57, 86, 74, 87, 90, 87,
49, 25, 61, 30, 73, 36, 86, 43, 98, 50, 41, 37, 53, 43, 66, 49, 79, 56, 92, 63, 34, 50, 46, 56, 59, 64, 72, 70, 84, 77, 27, 63, 39, 70, 51, 77, 64, 84, 77, 90,
36, 32, 47, 34, 59, 35, 73, 37, 87, 40, 32, 45, 43, 47, 56, 49, 69, 52, 84, 56, 29, 59, 40, 61, 52, 64, 65, 68, 80, 72, 26, 72, 37, 75, 48, 79, 62, 83, 75, 87,
28, 32, 43, 31, 60, 31, 76, 32, 92, 33, 26, 48, 41, 48, 58, 48, 75, 49, 93, 51, 25, 65, 40, 66, 57, 67, 75, 68, 92, 69, 24, 82, 40, 83, 56, 86, 73, 87, 90, 87
    };

    /*
    for(int i=0; i<BOARD_NBR; ++i)
    {
        for(int j=0; j<BOARD_HEIGHT; ++j)
        {
            for(int k=0; k<BOARD_WIDTH; ++k)
            {
                chosenPoints[i][j*BOARD_WIDTH +k].x = ptArray[2*(i*BOARD_HEIGHT*BOARD_WIDTH+j*BOARD_WIDTH +k)];
                chosenPoints[i][j*BOARD_WIDTH +k].y = ptArray[2*(i*BOARD_HEIGHT*BOARD_WIDTH+j*BOARD_WIDTH +k)+1];
            }


        }
        CV_MAT_ELEM( *point_counts, int, i, 0 ) = REQUIRED_CORNERS_COUNT;

    }
    */

    int cBrd = 0;
    while(cBrd<BOARD_NBR)
    {
        int indexNow = (cBrd)*REQUIRED_CORNERS_COUNT;                    // array stores the points of different frames, linearly
    for(int i=0; i<REQUIRED_CORNERS_COUNT; ++i)
            {
                int j = indexNow+i;
                CV_MAT_ELEM( *image_points, float, j, 0 ) = *(ptArray+2*i);
                CV_MAT_ELEM( *image_points, float, j, 1 ) = *(ptArray+2*i+1);
                CV_MAT_ELEM( *object_points, float, j, 0 ) = i/BOARD_WIDTH;                 // this assumes that the corner sent are in right order
                CV_MAT_ELEM( *object_points, float, j, 1 ) = i%BOARD_WIDTH;
                CV_MAT_ELEM( *object_points, float, j, 2 ) = 0.0f;                          // no Z-coordinate

            }
            CV_MAT_ELEM( *point_counts, int, cBrd, 0 ) = REQUIRED_CORNERS_COUNT;
            cBrd += 1;

    }


    // Start calibration
    getCalibrationParameters(CAMERA_LEFT);

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

    IplImage* cloneInput = cvCloneImage((IplImage*)inputImageLeft->getIplImage());
    cvRemap( cloneInput, (IplImage*)inputImageLeft->getIplImage(), mapx, mapy );

    for(int i=0; i<_poses; ++i)
    {
         char tmpName[20],saveName[20];
         sprintf(tmpName,"./rectified/R%d.ppm",i);
         sprintf(saveName,"./rectified/UnDis%d.ppm",i);
         IplImage* distIm = cvLoadImage(tmpName,CV_LOAD_IMAGE_GRAYSCALE);
         IplImage* cloneInput = cvCloneImage(distIm);
         cvRemap(distIm, cloneInput, mapx, mapy );
         cvSaveImage(saveName,cloneInput);
    }

}
void dvsCalibratorThread::run()
{

    //LMCurveFitter->initializeLM();
    //LMCurveFitter->solveLM();

    if(!resized)
    {
        resize(128,128);
        getRectifiedImages();
    }

    getchar();
    // Integrate the 500 images
    IplImage* integratedImage = cvCreateImage(cvSize(128,128),32,1);
    IplImage* integratedColorImage = cvCreateImage(cvSize(128,128),32,3);

    float* ptrIntImage = (float*)integratedImage->imageData;
    int wdt = integratedImage->widthStep /sizeof(float);

    count++;
    double dis1, dis2;
    int houghLineThickness = 1;
    int houghLineConnectivity = 8;
    double thetaResolution =(double)houghDThetaVal*exp(-1.0*(houghDThetaScale-5));
    double rhoResolution = (double)houghDRhoVal*exp(-1.0*(houghDRhoVal-5));
    char imageName[20];
    int intFrameCount = 100;
    while(saveOpt<1)
    {


/*
            IplImage* unDist = cvCloneImage(grayScale);
            IplImage* unDistC = cvCreateImage(cvGetSize(grayScale),8,3);
            cvSet(unDist,cvScalar(0));



            dis1 = exp(-1.0*sensitivityOfDistort[0])*(double)(distortionParameter[0]-500);
            dis2 = exp(-1.0*sensitivityOfDistort[1])*(double)(distortionParameter[1]-500);
            undistort(grayScale,unDist,dis1,dis2);
            cvCvtColor( unDist, unDistC, CV_GRAY2BGR );
            cvNamedWindow("undistortedImage");
            cvShowImage("undistortedImage",unDist);
            //printf("Current pars K1:%f K2%f\n",dis1,dis2);
            cvSaveImage("Rectified00001.ppm",unDist);
            cvCvtColor( grayScale, unDistC, CV_GRAY2BGR );


            //cvShowImage("Choose points",unDist);
            countOfBoards = countOfPoints = -1;
            renew = false;
            int totalCorners =0;
            do
            {
                if(renew)
                {
                    cvCvtColor( grayScale, unDistC, CV_GRAY2BGR );
                    cout<<"Board#"<<countOfBoards<<"  Corner Nbr#"<<countOfPoints<<endl;
                    renew = false;
                    for(int i=0; i<BOARD_NBR;++i)
                    {
                        for(int j=0; j<BOARD_HEIGHT*BOARD_WIDTH; ++j)
                        {
                            cout<<chosenPoints[i][j].x<<"  "<<chosenPoints[i][j].y<<"  ";
                        }
                        cout<<endl;
                    }
                }
                cvShowImage(nChoosingWindow,unDistC);
                cvWaitKey(1);
                if(countOfBoards >=0 && countOfPoints >=0 && (clickX!=0 || clickY !=0))
                {
                    countOfBoardNbr=countOfBoards;
                    (chosenPoints[countOfBoards][countOfPoints]).x=clickX;
                    (chosenPoints[countOfBoards][countOfPoints]).y=clickY;
                    int indexNow = (countOfBoards)*REQUIRED_CORNERS_COUNT;
                    int j = indexNow+countOfPoints;
                    CV_MAT_ELEM( *image_points, float, j, 0 ) = clickX;
                    CV_MAT_ELEM( *image_points, float, j, 1 ) = clickY;
                    CV_MAT_ELEM( *object_points, float, j, 0 ) = countOfPoints/BOARD_WIDTH;                 // this assumes that the corner sent are in right order
                    CV_MAT_ELEM( *object_points, float, j, 1 ) = countOfPoints%BOARD_WIDTH;
                    CV_MAT_ELEM( *object_points, float, j, 2 ) = 0.0f;                          // no Z-coordinate
                    cvCircle( unDistC, (chosenPoints[countOfBoards][countOfPoints]),1,CV_RGB(0,255,0));
                    totalCorners++;
                }



            }while(countOfBoards<BOARD_NBR);

            in++;


            //cvShowImage("Choose points",unDist);

            // Time to fit the lines
            IplImage* dst = cvCreateImage( cvGetSize(unDist), 8, 1 );
            IplImage* color_dst = cvCreateImage( cvGetSize(unDist), 8, 3 );
            CvMemStorage* storage = cvCreateMemStorage(0);
            CvSeq* lines = 0;

            int i;
            //10.0; // 1
            cvCanny( unDist, dst, 50, 200, 5 );
            cvCvtColor( dst, color_dst, CV_GRAY2BGR );


            if(houghMethod == 1)
            {
                lines = cvHoughLines2( dst, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 150, 0, 0 );

                for( i = 0; i < lines->total; i++ )
                {
                    float* line = (float*)cvGetSeqElem(lines,i);
                    float rho = line[0];
                    float theta = line[1];
                    CvPoint pt1, pt2;
                    double a = cos(theta), b = sin(theta);
                    if( fabs(a) < 0.001 )
                    {
                        pt1.x = pt2.x = cvRound(rho);
                        pt1.y = 0;
                        pt2.y = color_dst->height;
                    }
                    else if( fabs(b) < 0.001 )
                    {
                        pt1.y = pt2.y = cvRound(rho);
                        pt1.x = 0;
                        pt2.x = color_dst->width;
                    }
                    else
                    {
                        pt1.x = 0;
                        pt1.y = cvRound(rho/b);
                        pt2.x = cvRound(rho/a);
                        pt2.y = 0;
                    }
                    cvLine( color_dst, pt1, pt2, CV_RGB(255,0,0), houghLineThickness, houghLineConnectivity );
                }
            }
            else if(houghMethod == 2)
            {
                lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, rhoResolution, thetaResolution, houghThreshold, minLineLength, maxGapLength );
                for( i = 0; i < lines->total; i++ )
                {
                    CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
                    cvLine( color_dst, line[0], line[1], CV_RGB(255,0,0), houghLineThickness, houghLineConnectivity );

                    // find POI
                    CvPoint poiNow;
                    for(int j=i+1; j<lines->total; ++j)
                    {
                        CvPoint* line2 = (CvPoint*)cvGetSeqElem(lines,j);
                        if(findPOI(line[0], line[1], line2[0], line2[1], poiNow))
                        {
                            cvCircle( unDistC, poiNow,1,CV_RGB(0,255,0));
                        }
                    }

                }
            }
            else if(houghMethod == 3)
            {

                lines = cvHoughLines2( dst, storage, CV_HOUGH_MULTI_SCALE,rhoResolution, thetaResolution, houghThreshold, minLineLength, maxGapLength);
                printf("Number of LINES: %d\n",lines->total);

                for( i = 0; i < lines->total; i++ )
                {
                    float* line = (float*)cvGetSeqElem(lines,i);
                    float rho = line[0];
                    float theta = line[1];
                    CvPoint pt1, pt2;
                    double a = cos(theta), b = sin(theta);
                    if( fabs(a) < 0.001 )
                    {
                        pt1.x = pt2.x = cvRound(rho);
                        pt1.y = 0;
                        pt2.y = color_dst->height;
                    }
                    else if( fabs(b) < 0.001 )
                    {
                        pt1.y = pt2.y = cvRound(rho);
                        pt1.x = 0;
                        pt2.x = color_dst->width;
                    }
                    else
                    {
                        pt1.x = 0;
                        pt1.y = cvRound(rho/b);
                        pt2.x = cvRound(rho/a);
                        pt2.y = 0;
                    }
                    cvLine( color_dst, pt1, pt2, CV_RGB(255,0,0), houghLineThickness, houghLineConnectivity );
                }

            }
            cvNamedWindow( "Source");
            cvShowImage( "Source", unDistC );

            cvNamedWindow( "Hough");
            cvShowImage( "Hough", color_dst );
            cvWaitKey(20);
*/
        }
        getchar();


        if(saveOpt>=1)
        {
            CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = exp(-1.0*sensitivityOfFocalLength[0])*(double)(focalLengths[0]-500);
            CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = exp(-1.0*sensitivityOfFocalLength[1])*(double)(focalLengths[1]-500);
            CV_MAT_ELEM( *intrinsic_matrix, float, 0, 2 ) = imageCenter[0]; // this is cx
            CV_MAT_ELEM( *intrinsic_matrix, float, 1, 2 ) = imageCenter[1]; // this is cy
            CV_MAT_ELEM( *distortion_coeffs, float, 0, 0 ) = exp(-1.0*sensitivityOfDistort[0])*(double)(distortionParameter[0]-500);
            CV_MAT_ELEM( *distortion_coeffs, float, 1, 0 ) = exp(-1.0*sensitivityOfDistort[1])*(double)(distortionParameter[1]-500);
            CV_MAT_ELEM( *distortion_coeffs, float, 2, 0 ) = 0;//P1;
            CV_MAT_ELEM( *distortion_coeffs, float, 3, 0 ) = 0;//P2;

            //getCalibrationParameters(CAMERA_LEFT,intrinsic_matrix,distortion_coeffs);

            // should cleanup and stop the thread
            exit(0);


        }
        //fprintf(distortParams,"K1:%f\nK2:%f\n",dis1,dis2);
        //fprintf(distortParams,"Method%d\ndRho:%f\ndTheta%f\nThreshold%d Min Length:%d\n max gap: %d\n",rhoResolution, thetaResolution, houghThreshold, minLineLength, maxGapLength);

    fclose(distortParams);

    IplImage* checkImage2 = cvCreateImage(cvSize(128,128),8,1);

    //cvSet(checkImage, cvScalar(0));
    int wdCheck2 = checkImage2->widthStep;

    gsl_vector* retVect;
    fitTheData(Ycoord,retVect);

    double fitR = gsl_vector_get(s->x, 0);
    double fitPhi = gsl_vector_get(s->x, 1);
    double fitK1 = gsl_vector_get(s->x, 2)*SCALE_K1;
    //double fitK2 = gsl_vector_get(s->x,3)*SCALE_K2;

    //fitR = 40.0/1.4142;
    //fitPhi = PI/4.0;
    //fitK1 =  -.005;//  .3/(64.0*64.0);
    //fitK2 = .3/(64.0*64.0*64.0*64.0);

    IplImage* checkImage3 = cvCreateImage(cvSize(128,128),8,1);
    uchar* ptrCheck3 = (uchar*)checkImage3->imageData;
    cvSet(checkImage3, cvScalar(0));
    int wdCheck3 = checkImage3->widthStep;

    for(int i=0; i<N; ++i)
    {
        double fittedRNow = fitR / cos(Xcoord[i]- fitPhi);//1.0 + 5 * exp (-0.1 * t)  * gsl_ran_gaussian( r, .1)
        //+ gsl_ran_gaussian (r, 0.1);(1.0+gsl_ran_gaussian (r, 0.03)) *
        //sigma[i] = 0.1;
        fittedRNow = fittedRNow*(1.0 + fitK1*fittedRNow);//*fittedRNow + fitK2*fittedRNow*fittedRNow*fittedRNow*fittedRNow);
        int Xc = max(0,min(128,64- (int)(fittedRNow*sin(Xcoord[i]))));
        int Yc = max(0,min(128,64 + (int)(fittedRNow*cos(Xcoord[i]))));
        *(ptrCheck3 + wdCheck3*Xc + ( Yc)) = 255;

    }
    cvNamedWindow("fittedLine");
    cvShowImage("fittedLine",checkImage3);
    cvWaitKey(2);


    while((!calibrationDoneForLeftCamera) ) //&& inputPortLeft.getInputCount()){
    {
        //printf("In calibration for left \n");
        isLeftCameraActive = true;
        //inputImageLeft   = inputPortLeft.read(true);
        //inputImageLeft->zero();
        //printf("Fetching the current image \n");
        if(spatialFrameCreatorThread->spatialFrameCreatorLeft->isSFCreatorInitialized() && (!(spatialFrameCreatorThread->spatialFrameCreatorLeft->getIsAccessing())))
        {
            spatialFrameCreatorThread->spatialFrameCreatorLeft->setIsAccessing(true);
            //while(spatialFrameCreator->getIsUpdating()) ;
            inputImageLeft = spatialFrameCreatorThread->getTheImage(true); //spatialFrameCreatorLeft->getMonoImage();
            //printf("Got the mono image \n");
            spatialFrameCreatorThread->spatialFrameCreatorLeft->setIsAccessing(false);

            //if(cvWaitKey(20) == 27) {
            IplImage* copyIm = cvCloneImage((IplImage*)inputImageLeft->getIplImage());
            cvSaveImage("imageRcvd.jpg",copyIm);
            //}
        }
        else
        {
            printf("Got NO mono image \n");
            continue;
        }
        if(inputImageLeft != NULL)
        {
            if(!resized)
            {
                printf("Resizing\n");
                resize(128,128);//inputImageLeft->width(),inputImageLeft->height());
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
            printf("Find corners or integrate\n");
            if(count%refreshOfSummationRate == 0)
            {
                findCorners(CAMERA_LEFT);
                outputPortLeft.prepare()= *tempVariation;
                outputPortLeft.write();
                tempVariation->zero();
            }
            else
            {
                printf("integrate\n");
                integrateCurrentFrame(inputImageLeft);
            }
        }


    }
    if(calibrationDoneForLeftCamera)     // free some resources for next camera in the queue
    {
        count = 0;
        tempVariation->zero();
    }


}

bool dvsCalibratorThread::findPOI(CvPoint P11, CvPoint P12,CvPoint P21, CvPoint P22, CvPoint& retVal)
{

    CvPoint x, d1, d2;
    x.x = P21.x - P11.x;
    d1.x = P11.x - P12.x;
    d2.x = P21.x- P22.x;
    x.y = P21.y - P11.y;
    d1.y = P11.y - P12.y;
    d2.y = P21.y- P22.y;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;



    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    retVal.x = P11.x + d1.x * t1;
    retVal.y = P11.y + d1.y * t1;

    // Must also check if POI lies within polygon!
    if(retVal.x<0 || retVal.x>128 || retVal.y<0 || retVal.y>128)
        return false;

    return true;


}

bool dvsCalibratorThread::fitTheData(double* valueOfPoints, gsl_vector* fittedParams)
{


    n= N;
    p = NBR_PARAMETERS;

    covar = gsl_matrix_alloc (NBR_PARAMETERS, NBR_PARAMETERS);
    double y[N], sigma[N];
    struct dataE d = { n, y, sigma};

    double x_init[NBR_PARAMETERS] = { 40.99/1.4142, PI/4.0, -.399/(64.0)};//*64.0)};//,.39/(64.0*64.0*64.0*64.0) };
    gsl_vector_view x = gsl_vector_view_array (x_init, NBR_PARAMETERS);
    const gsl_rng_type * type;


    f.f = &modelFunction;
    f.df = &derivativeModelFunction;
    f.fdf = &modelFunctionAndDerivative;
    f.n = n;
    f.p = NBR_PARAMETERS;
    f.params = &d;

    /* This is the data to be fitted */


    for (int i = 0; i < n; i++)
    {
        y[i] = Ycoord[i];
        double t = i;
        //y[i] = 1.0 + 5 * exp (-0.1 * t)
        //+ gsl_ran_gaussian (r, 0.1);
        //sigma[i] = 0.1;
        //printf ("data: %u %g %g\n", i, y[i], sigma[i]);
    };

    T = gsl_multifit_fdfsolver_lmsder;
    s = gsl_multifit_fdfsolver_alloc (T, n, NBR_PARAMETERS);
    gsl_multifit_fdfsolver_set (s, &f, &x.vector);

    print_state (iter, s);

    do
    {
        iter++;
        status = gsl_multifit_fdfsolver_iterate (s);

        printf ("status = %s\n", gsl_strerror (status));

        print_state (iter, s);

        if (status)
            break;

        status = gsl_multifit_test_delta (s->dx, s->x,
                                          TOLERANCE_DERIVATIVE_FN, TOLERANCE_FN);
    }
    while (status == GSL_CONTINUE && iter < 1000);

    gsl_multifit_covar (s->J, 0.0, covar);

#define FIT(i) gsl_vector_get(s->x, i)
#define ERR(i) sqrt(gsl_matrix_get(covar,i,i))

    {
        double chi = gsl_blas_dnrm2(s->f);
        double dof = N - NBR_PARAMETERS;
        double c = GSL_MAX_DBL(1, chi / sqrt(dof));

        printf("chisq/dof = %f\n",  pow(chi, 2.0) / dof);

        printf ("R0      = %f +/- %f\n", FIT(0), c*ERR(0));
        printf ("phi = %f +/- %f\n", FIT(1), c*ERR(1));
        printf ("K1     = %f +/- %f\n", FIT(2), c*ERR(2));
        //printf ("K2     = %f +/- %f\n", FIT(3), c*ERR(3));
    }

    printf ("status = %s\n", gsl_strerror (status));
    fittedParams = s->x;

    gsl_multifit_fdfsolver_free (s);
    gsl_matrix_free (covar);
    //gsl_rng_free (r);

    // return false if fit is too bad

    return true;

}

void dvsCalibratorThread::prepareTheData(IplImage* inputImage)
{

    uchar* ptrInputImage = (uchar*)inputImage->imageData;
    uchar* ptrHeadImage = (uchar*)inputImage->imageData;
    int widthInputImage = inputImage->widthStep/ sizeof(uchar);
    int WHITE = 130;
    int depth = 11;
    const int totPts = 500;
    double thetaVals[totPts];
    double rhoVals[totPts];
    int H = inputImage->height;
    int W = inputImage->width;
    int whitePixels[128*128/10]; // dont expect more than 10% of image to be white
    memset(whitePixels,0,128*128*sizeof(uchar)/10);
    int cnt = 0;
    bool isGoodWhite;
    for(int i=0; i<H; ++i)
    {
        for(int j=0; j<W; ++j)
        {

            if(*(ptrHeadImage + i*widthInputImage + j) >WHITE)  //is white
            {
                whitePixels[cnt++] =i*128+j;
                /*isGoodWhite = false;
                int sgn1[4] = {-1,-1,1,1};
                int sgn2[4] = {-1,1,-1,1};
                int regionCount[4] = {0,0,0,0};
                for(int r=0; r<4; ++r){
                    for(int k =1; k<depth; ++k){
                        for(int l = 1; l<depth; ++l){
                           if(*(ptrHeadImage + (i+sgn1[r]*k)*widthInputImage + j+sgn2[r]*l) >WHITE ){
                            regionCount[r] += 1;
                           }
                        }
                    }
                    if(regionCount[r]> .2* depth* depth){
                        whitePixels[cnt] =i*128+j;
                        isGoodWhite = true;
                    }
                }
                if(isGoodWhite && cnt < totPts) {
                    float yVal = (float)(i-64);
                    float xVal = (float)(j-64);
                    thetaVals[cnt] = atan(yVal/xVal);
                    rhoVals[cnt] = sqrt(yVal*yVal + xVal*xVal);
                    cnt++;
                }*/
            }


        }
    }
    cnt--;
    printf("CNT%d\n",cnt);

    // let us see
    IplImage* checkImage = cvCreateImage(cvGetSize(inputImage),8,1);
    uchar* ptrCheck = (uchar*)checkImage->imageData;
    //cvSet(checkImage, cvScalar(0));
    int wdCheck = checkImage->widthStep;
    IplImage* checkImage2 = cvCreateImage(cvGetSize(inputImage),8,1);
    uchar* ptrCheck2 = (uchar*)checkImage2->imageData;
    //cvSet(checkImage, cvScalar(0));
    int wdCheck2 = checkImage->widthStep;

    for(int i=0; i<cnt-1 && i<N; ++i)
    {
        int row = (whitePixels[i]/128);
        int col = (whitePixels[i] %128);
        float yVal = (float)(64-row);
        float xVal = (float)(col-64);
        Xcoord[i] = atan2(yVal,xVal);
        Ycoord[i] = sqrt(yVal*yVal + xVal*xVal);

        printf("row:%d, col:%d, yVal%f, xVal%f, form1%f, Xcoord:%f, Ycoord:%f \n",row,col,yVal,xVal,atan2(yVal,xVal),Xcoord[i], Ycoord[i]);
        *(ptrCheck + wdCheck*row  + col) = *(ptrHeadImage + row*widthInputImage + col);
        *(ptrCheck2 + wdCheck*(64- (int)(Ycoord[i]*sin(Xcoord[i]))) + (64+(int)(Ycoord[i]*cos(Xcoord[i])))) = *(ptrHeadImage + row*widthInputImage + col);

    }
    //cvNamedWindow("checkImg");
    //cvShowImage("checkImg",checkImage);
    //cvNamedWindow("checkImg2");
    //cvShowImage("checkImg2",checkImage2);
    //cvWaitKey(0);





}

void dvsCalibratorThread::findCorners(int whichCamera)
{

    printf("\nFindingCorners\n");

    now = (IplImage*)tempVariation->getIplImage();
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
    for(int i=0;  i<now->height-windowHeight; ++i)
    {
        for(int j=0; j<now->width-windowWidth; ++j)
        {
            countForCandidatePixels[i][j] = 0;
        }
    }
    for(int i=windowHeight; i<now->height-windowHeight; ++i)
    {
        for(int j=windowWidth; j<now->width-windowWidth; ++j)
        {
            int valueForCurrentWindow = findBlob(now,j,i,expectedWindowSize[0],expectedWindowSize[1],thresholdForWhite);
            if( valueForCurrentWindow > filled)
            {
                *(ptrOut+i*actualWidth+j) = 255;
            }
            else
            {
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
    for(int i=0; i<centroidImage->height; ++i)
    {
        for(int j=0; j<centroidImage->width; ++j)
        {
            traversalTable[i*IMG_WIDTH + j] = false;
        }
    }
    binCount = 0;
    for(int i=0; i<BOARD_WIDTH*BOARD_HEIGHT; ++i)
    {
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
    if(nbrOfBoardsVisited< BOARD_NBR)
    {
        prepareToCalibrate(binCount,arrayOfCentroids);

    }
    else if(nbrOfBoardsVisited == BOARD_NBR)            // we are ready for calibration
    {
        getCalibrationParameters(whichCamera);

    }
    printf("\nFound Corners\n");

}

void dvsCalibratorThread::getCalibrationParameters(int whichCamera)
{

        printf("Calibration begun for camera#%d!\n",whichCamera);
        cvCalibrateCamera2( object_points, image_points, point_counts, cvGetSize( centroidImage ),
                            this->intrinsic_matrix, this->distortion_coeffs, NULL, NULL, CV_CALIB_FIX_PRINCIPAL_POINT);  // you need 2 matrices: object_point2 which has coordinate + 1 dimension for all corner points across all showings
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
    fprintf(paramsFilePtr,"[CAMERA_CALIBRATION_DVS#%d]\n\n",whichCamera);
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

    if(whichCamera == CAMERA_LEFT)
    {
        calibrationDoneForLeftCamera = true;
    }
    else if(whichCamera == CAMERA_RIGHT)
    {
        calibrationDoneForRightCamera = true;
    }
    else
    {
        // INVALID for now
    }
}

void dvsCalibratorThread::prepareToCalibrate(int nbrOfCorners, int* arrayOfCorners)
{

    /*
    // Calibrate the camera
    cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ),
    	intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );  // you need 2 matrices: object_point2 which has coordinate + 1 dimension for all corner points across all showings
    	                                                                                // and image_point2 which
    	                                                                                // has coordinate dimension for all corners across all showing
    */
    // check the required nbr of points
    if(nbrOfCorners != REQUIRED_CORNERS_COUNT)
    {
        //printf("Not exact points, %d nbr of points were detected! \n",nbrOfCorners);
        return;
    }
    else
    {
        printf("Going to use this board!This is %d th board visited Press esc to refuse and return to accept\n",nbrOfBoardsVisited);
        //IplImage* tmpImage = cvCreateImage(cvSize(128,128),8,1);
        char val=cvWaitKey(0);
        if(val==13 || val=='y')
        {
            for(int i=0; i<REQUIRED_CORNERS_COUNT; ++i)
            {
                printf(":%d,%d\t",*(arrayOfCorners+2*i),*(arrayOfCorners+2*i+1));
            }
            printf("\n\n");

            int indexNow = (nbrOfBoardsVisited)*REQUIRED_CORNERS_COUNT;                    // array stores the points of different frames, linearly
            for(int i=0; i<REQUIRED_CORNERS_COUNT; ++i)
            {
                int j = indexNow+i;
                CV_MAT_ELEM( *image_points, float, j, 0 ) = *(arrayOfCorners+2*i);
                CV_MAT_ELEM( *image_points, float, j, 1 ) = *(arrayOfCorners+2*i+1);
                CV_MAT_ELEM( *object_points, float, j, 0 ) = i/BOARD_WIDTH;                 // this assumes that the corner sent are in right order
                CV_MAT_ELEM( *object_points, float, j, 1 ) = i%BOARD_WIDTH;
                CV_MAT_ELEM( *object_points, float, j, 2 ) = 0.0f;                          // no Z-coordinate

            }
            CV_MAT_ELEM( *point_counts, int, nbrOfBoardsVisited, 0 ) = REQUIRED_CORNERS_COUNT;
            nbrOfBoardsVisited += 1;
        }
        else if(val==27)
        {
            //neglect
        }
        return;
    }

}

void dvsCalibratorThread::visitAllNodes(IplImage* imageGraph, IplImage* imageWithCentroidsMarked, int* posToVisit)
{

    uchar* rootNode = (uchar*)imageGraph->imageData;
    uchar* centroid = (uchar*)imageWithCentroidsMarked->imageData;

    int actualWidth = imageGraph->widthStep;
    int actualWidthCentroid = imageWithCentroidsMarked->widthStep;

    int jump =2;

    // Base case of recursion
    if(posToVisit[0]>imageGraph->height || posToVisit[1]>imageGraph->width || traversalTable[posToVisit[0]*IMG_WIDTH + posToVisit[1]])
    {
        return;
    }
    // Recurse!
    else
    {
        int tempPos[2];
        tempPos[0]=tempPos[1]=totalMoment[0]=totalMoment[1]=countOfPointsInBlob = 0;
        visitChildNode(imageGraph,posToVisit, tempPos);
        if(countOfPointsInBlob>1)
        {
            binCount++;
            int centroidX = totalMoment[0]/countOfPointsInBlob;
            int centroidY = totalMoment[1]/countOfPointsInBlob;
            *(centroid+actualWidthCentroid*centroidX+centroidY) = 255;
            if(binCount<=BOARD_HEIGHT*BOARD_WIDTH)
            {
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


void dvsCalibratorThread::visitChildNode(IplImage* imageGraph,int* presentPos, int* nextPos)
{

    uchar* rootNode = (uchar*)imageGraph->imageData;
    int actualWidth = imageGraph->widthStep;
    // Base case of recursion
    if(presentPos[0]>imageGraph->height || presentPos[1]>imageGraph->width || *(rootNode+actualWidth* (presentPos[0]) + presentPos[1]) == 0
            || traversalTable[presentPos[0]*IMG_WIDTH + presentPos[1]])
    {
        return; // do nothing and terminate recursion
    }
    // Recurse!
    else
    {
        countOfPointsInBlob++;
        totalMoment[0] += presentPos[0];
        totalMoment[1] += presentPos[1];
        // store max of all X and Y positions in this region in nextPos
        if(presentPos[0] > nextPos[0])
        {
            nextPos[0] = presentPos[0];
        }
        if(presentPos[1] > nextPos[1])
        {
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

int dvsCalibratorThread::findBlob(IplImage* origImgFB,int posWidth,int posHeight,  int w, int h, int threshold)
{

    int countOfPoints = 0;
    int imageWidth = origImgFB->widthStep;
    uchar* startingPtr = (uchar*)origImgFB->imageData;
    uchar* currentPtr = startingPtr + imageWidth*posHeight + posWidth;
    for(int i=0; i<h; ++i)
    {
        currentPtr = startingPtr + imageWidth*(i+posHeight) + posWidth;
        for(int j=0; j<w; ++j)
        {
            if(*currentPtr > threshold)
            {
                countOfPoints++;
            }
            currentPtr++;
        }
    }
    return countOfPoints;
}

void dvsCalibratorThread::integrateCurrentFrame(ImageOf<PixelMono>* currentFrame)
{
    printf("Integrating \n");
    uchar* ptrCurrentImage = (uchar*)currentFrame->getRawImage();
    uchar* ptrTmpVariation = (uchar*)tempVariation->getRawImage();
    int padCurrentImage = currentFrame->getPadding();
    float weightageOfFrame = 1.0/(255.0);
    float wtForVal;
    float decayRate = 10.0;

    for(int i=0; i<currentFrame->height(); ++i)
    {
        for(int j=0; j<currentFrame->width(); ++j)
        {
            uchar val = *ptrCurrentImage;
            if(val > 238 && *ptrTmpVariation < 238)
            {
                *ptrTmpVariation = 255;
            }

            ptrCurrentImage++;
            ptrTmpVariation++;
        }
        ptrCurrentImage         += padCurrentImage;
        ptrTmpVariation         += padCurrentImage;
    }



}

void dvsCalibratorThread::threadRelease()
{
    printf("freeing memory in integrator");
    delete spatialFrameCreatorThread;
#ifdef USE_OPEN_CV_WINDOW
    cvDestroyWindow("imageNow");
    cvDestroyWindow("centroidWindow");
    cvDestroyWindow("out");
#endif
    //inputPortLeft.interrupt();
    //inputPortRight.interrupt();
    outputPortLeft.interrupt();
    outputPortRight.interrupt();
    //inputPortLeft.close();
    //inputPortRight.close();
    outputPortLeft.close();
    outputPortRight.close();

    delete inputImageLeft;
    delete inputImageRight;
    delete tempVariation;
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

