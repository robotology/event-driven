//
// Created by miacono on 11/07/17.
//

#include <vMapping.h>


using namespace ev;
using namespace yarp::math;

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find yarp network";
        return 1;
    }
    
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "cameraCalibration" );
    rf.setDefaultConfigFile( "vMapping.ini" );
    rf.configure( argc, argv );
    
    vMappingModule mappingModule;
    return mappingModule.runModule(rf);
}

bool vMappingModule::updateModule() {
    
    //if calibrate parameter is set the update will perform a calibration step maxIter times
    if (calibrate) {
        if ( imageCollector.isImageReady() && vImageCollector.isImageReady() ) {
            yarp::sig::ImageOf<yarp::sig::PixelBgr> frame = imageCollector.getImage();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> vImg = vImageCollector.getImage();
        
            auto *frameIplImg = (IplImage *) frame.getIplImage();
            cv::Mat frameMat = cv::cvarrToMat( frameIplImg );
            cv::imshow( "img", frameMat );
        
            auto *vIplImg = (IplImage *) vImg.getIplImage();
            cv::Mat vMat = cv::cvarrToMat( vIplImg );
            cv::imshow( "vImg", vMat );
            cv::waitKey( 1 );
            cv::Size boardSize( 4, 11 );
            std::vector<cv::Point2f> frameCenters; //Vector for storing centers of circle grid on frame image
            std::vector<cv::Point2f> vCenters; //Vector for storing centers of circle grid on event image
        
            bool frameFound = cv::findCirclesGrid( frameMat, boardSize, frameCenters,
                                               cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING );
            bool eventFound = cv::findCirclesGrid( vMat, boardSize, vCenters,
                                               cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING );
                
            if ( frameFound && eventFound ) {
                cvCvtColor( frameIplImg, frameIplImg, CV_RGB2BGR );
                cvCvtColor( vIplImg, vIplImg, CV_RGB2BGR );

                cv::Mat frameCentersMat( frameCenters );
                cv::Mat vCentersMat( vCenters );
                drawChessboardCorners( frameMat, boardSize, frameCentersMat, frameFound );
                drawChessboardCorners( vMat, boardSize, vCentersMat, eventFound );
                cv::Mat h = cv::findHomography( vCenters, frameCenters, CV_RANSAC );
                cv::imshow( "img", frameMat );
                cv::imshow( "vImg", vMat );
                cv::waitKey( 1000 );
                for (int r = 0; r < homography.rows(); ++r) {
                    for (int c  = 0; c  < homography.cols(); ++c ) {
                        homography(r,c) += h.at<double>(r,c);
                    }
                }
                nIter++;
                
                //When max number of iteration is reached calibration is finalized
                if (nIter >= maxIter){
                    homography /= nIter;
                    homography = homography.transposed();
                    cv::destroyAllWindows();
                    yInfo() << "Calibration is over after " << nIter << " steps";
                    calibrate = false;
                }
            }
        
        }
    
        return true;
    }
    
    //If image is ready remap and draw events on it
    if (imageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr > &frame = imagePortOut.prepare();
        frame = imageCollector.getImage();
        ev::vQueue vQueue = eventCollector.getEvents();
        for (auto it = vQueue.begin(); it != vQueue.end(); ++it){
            
            auto v = ev::is_event<ev::AE >(*it);
            if (v->channel) continue; //Skipping events from right cam
            double x = (303 - v->x);
            double y = (239 - v->y);
            yarp::sig::Vector evCoord (3);
            
            //Converting to homogeneous coordinates
            evCoord[0] = x;
            evCoord[1] = y;
            evCoord[2] = 1;
            
            //Applying trasformation
            evCoord *= homography;
            
            //Converting back from homogenous coordinates
            x = evCoord[0] / evCoord[2];
            y = evCoord[1] / evCoord[2];
            
            //Drawing event on img
            if (x >= 0 && x < frame.width())
                if (y >= 0 && y < frame.height())
                    frame(x,y) = yarp::sig::PixelBgr(255,255,255);
        }
        imagePortOut.write();
    }
    
    return true;

   
}

bool vMappingModule::configure( yarp::os::ResourceFinder &rf ) {
    std::string moduleName = rf.check("name",yarp::os::Value("/vMapping")).asString();
    calibrate = rf.check("calibrate", yarp::os::Value(false)).asBool();
    this -> outFileName = rf.getHomeContextPath().c_str();
    outFileName += "/vMapping.ini";

    yInfo() << "fileName = " << outFileName.c_str();
    yInfo() << "ASdjfafksnfdka";
    setName(moduleName.c_str());
    homography.resize(3,3);

    imageCollector.open(getName("/img:i"));
    
    if (calibrate) {
        vImageCollector.open( getName( "/vImg:i" ) );
        maxIter = rf.check( "maxIter", yarp::os::Value( 20 ) ).asInt();
        nIter = 0;
    }
    
    eventCollector.open(getName("/vBottle:i"));
    imagePortOut.open(getName("/img:o"));
    eventCollector.start();
    imageCollector.start();
    vImageCollector.start();
    return true;
}

bool vMappingModule::interruptModule() {
    imagePortOut.interrupt();
    imageCollector.interrupt();
    vImageCollector.interrupt();
    eventCollector.interrupt();
    return true;
}

bool vMappingModule::close() {
    imagePortOut.close();
    imageCollector.close();
    vImageCollector.close();
    eventCollector.close();
    return true;
}

double vMappingModule::getPeriod() {
    return 0;
}

/**************************ImagePort*********************************/

yarp::sig::ImageOf<yarp::sig::PixelBgr> ImagePort::getImage(){
    yarp::sig::ImageOf<yarp::sig::PixelBgr> outImg;
    mutex.lock();
    outImg = image;
    imageReady = false;
    mutex.unlock();
    return outImg;
};

void ImagePort::onRead( yarp::sig::ImageOf<yarp::sig::PixelBgr> &inImg ){
    mutex.lock();
    image = inImg;
    imageReady = true;
    mutex.unlock();
};

/***********************EventBottleManager***********************/

EventBottleManager::EventBottleManager() {
    
    //here we should initialise the module
    vCount = 0;
    latestStamp = 0;
    isReading = false;
}

bool EventBottleManager::open(const std::string &name) {
    //and open the input port
    
    this->useCallback();
    
    BufferedPort<ev::vBottle>::open(name);
    return true;
}

void EventBottleManager::onRead(ev::vBottle &bot) {
    if (!isReading)
        return;
    
    //get new events
    ev::vQueue newQueue = bot.get<ev::AE>();
    if(newQueue.empty()){
        std::cout << "Empty queue" << std::endl;
        return;
    }
    
    mutex.wait();
    //append new events to queue
    vQueue.insert(vQueue.end(), newQueue.begin(), newQueue.end());
    //latestStamp = unwrapper(newQueue.back()->stamp);
    //vCount += newQueue.size();
    mutex.post();
}

unsigned long int EventBottleManager::getTime() {
    return latestStamp;
    
}

unsigned long int EventBottleManager::popCount() {
    mutex.wait();
    unsigned long int r = vCount;
    vCount = 0;
    mutex.post();
    return r;
    
}

bool EventBottleManager::start() {
    mutex.wait();
    vQueue.clear();
    isReading = true;
    yRate = yarp::os::Time::now();
    mutex.post();
    return true;
}

bool EventBottleManager::stop() {
    mutex.wait();
    isReading = false;
    yRate = yarp::os::Time::now() - yRate;
    yRate = vCount / yRate;
    vCount = 0;
    mutex.post();
    return true;
}

ev::vQueue EventBottleManager::getEvents() {
    mutex.wait();
    if (!&vQueue)
        return ev::vQueue();
    ev::vQueue outQueue = vQueue;
    vQueue.clear();
    mutex.post();
    
    return outQueue;
}
