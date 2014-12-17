// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <stdlib.h>


#include <cv.h>
#include <highgui.h>

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#define COMMAND_VOCAB_ON    VOCAB2('o','n')
#define COMMAND_VOCAB_OFF   VOCAB3('o','f','f')
#define COMMAND_VOCAB_DUMP  VOCAB4('d','u','m','p')
#define COMMAND_VOCAB_SYNC  VOCAB4('s','y','n','c')

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace cv;


inline const void  doubleSize(const char* pathIn, const char* pathOut, const int xDim, const int yDim){
    cv::Mat image;
    image = imread( pathIn, 1 );
    
    cv::Mat gray_image;
    cvtColor( image, gray_image, CV_BGR2GRAY );

    
    
    int heightDouble = yDim << 1;
    int widthDouble  = xDim << 1;
    printf("heightDouble %d widthDouble %d \n",heightDouble,widthDouble );
    IplImage* imageC = cvCreateImage(cvSize(widthDouble, heightDouble),IPL_DEPTH_8U ,3);
    //Mat M(widthDouble, heightDouble, CV_64F);
    cv::Mat M = imageC;

    Vec3b point, pointA, pointB;

    printf("doubleSize %d %d in %s %s \n",xDim,yDim, pathIn, pathOut);
    for (int r = 1; r < heightDouble; r ++){
        for (int c = 1; c < widthDouble; c++){
            if(( c % 2 == 0)&&(r % 2 == 0)) {
                // M.at<double>(r,c) = image.at<double>(r>>1, c>>1);
                point =  image.at<Vec3b>(r>>1,c>>1);
                M.at<Vec3b>(r,c) = point;
            }
            else {
                
                    pointA    = image.at<Vec3b>(r>>1, (c>>1) + 1);
                    pointB    = image.at<Vec3b>(r>>1, c>>1);
                    int sum0  = pointA.val[0] + pointB.val[0];
                    int sum1  = pointA.val[1] + pointB.val[1];
                    int sum2  = pointA.val[2] + pointB.val[2];
                    Vec3b pointMean(sum0 >> 1, sum1 >> 1, sum2 >> 1);
                    M.at<Vec3b>(r,c) = pointMean;
                
            }
        }
    }
    
    imwrite( pathOut, M );
    cvReleaseImage(&imageC);
    
}


int main(int argc, char *argv[]) 
{
    Network yarp; yarp::os::Network::init();

    //Port* _pOutPort = new Port;
    //_options.portName+="/command:o";
    //std::string portName="/simpleSaccade/cmd:o";
    //_pOutPort->open(portName.c_str());

    Property params;
    params.fromCommand(argc, argv);
    if(params.check("help"))
    {
        fprintf(stderr, "%s --robot robotName --loop numberOfLoop", argv[0]);
    }
    
    if (!params.check("xDim"))
    {
        fprintf(stderr, "Please specify the name of the xDim \n");
        fprintf(stderr, "--xDim (e.g. icub)\n");
        return -1;
    }
    if (!params.check("yDim"))
    {
        fprintf(stderr, "Please specify the yDim\n");
        fprintf(stderr, "--yDim\n");
        return -1;
    }

    //std::string robotName=params.find("robot").asString().c_str();
    //std::string remotePorts="/";
    //remotePorts+=robotName;
    //remotePorts+="/head"; //"/right_arm"

    //int nOl=atoi(params.find("loop").asString().c_str());
    int xDim = params.find("xDim").asInt();
    int yDim = params.find("yDim").asInt();

    int maxIndex =  365;

    
    std::string pathIn("/tmp/data/");
    std::string pathOut    = "/tmp/dataDouble/";
    std::string exten    = ".jpg";
    std::string pf       = "0000000";
    
    
    char pathFileIn[50];
    char pathFileOut[50];
    char* ppathFileIn  = &pathFileIn[0];
    char* ppathFileOut = &pathFileOut[0];
    
    printf("pointers %08X %08X \n", ppathFileIn, ppathFileOut);
    
    for (int i = 0; i < 10 & i <= maxIndex; i++) {
        sprintf(ppathFileIn ,"%s%s%d%s",pathIn.c_str(),pf.c_str(), i, exten.c_str()); 
        sprintf(ppathFileOut,"%s%s%d%s",pathOut.c_str(),pf.c_str(),i,exten.c_str());
        printf("pathFile: %s \n", ppathFileIn);
        doubleSize(ppathFileIn, ppathFileOut, xDim, yDim);
    }
    
    
    /*
    pf = "00000";
    for (int i = 10; i < 1000 & i <= maxIndex; i++) {
        sprintf((char*)pathFileIn.c_str() ,"%s%s%d%s",pathIn.c_str(),pf.c_str(),i,exten.c_str()); 
        sprintf((char*)pathFileOut.c_str(),"%s%s%d%s",pathOut.c_str(),pf.c_str(),i,exten.c_str());
        printf("pathFile: %s \n", pathFileIn.c_str());
        doubleSize(pathFileIn.c_str(), pathFileOut.c_str(), xDim, yDim);
    }
    */
    

    printf("Closing the executable; Freeing memory \n");
    yarp::os::Network::fini();
    
   
    return 0;
}
