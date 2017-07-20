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
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vMapping.ini" );
    rf.configure( argc, argv );
    
    vMappingModule mappingModule;
    return mappingModule.runModule(rf);
}

bool vMappingModule::updateModule() {
    
    if (imageCollector.isImageReady() && vImageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr> frame = imageCollector.getImage();
        yarp::sig::ImageOf<yarp::sig::PixelBgr> vImg = vImageCollector.getImage();
        
        IplImage* imgL= (IplImage*) frame.getIplImage();
        cv::Mat mat= cv::cvarrToMat(imgL);
        cv::imshow("img",mat);
        
        IplImage* vImgL= (IplImage*) vImg.getIplImage();
        cv::Mat vMat= cv::cvarrToMat(vImgL);
        cv::imshow("vImg",vMat);
        
        cv::waitKey(1);
        cv::Size boardSize (4 , 11);
        std::vector <cv::Point2f> pointbufL;
        std::vector <cv::Point2f> pointbufR;
        
        bool foundL = cv::findCirclesGrid(mat, boardSize, pointbufL, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
        bool foundR = cv::findCirclesGrid(vMat, boardSize, pointbufR, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
        
        if (foundL) std::cout << "frame" << std::endl;
        if (foundR) std::cout << "events" << std::endl;
        if(foundL && foundR) {
            std::cout << "found" << std::endl;
            cvCvtColor(imgL,imgL,CV_RGB2BGR);
            cvCvtColor(vImgL,vImgL, CV_RGB2BGR);
//            cv::saveStereoImage(pathImg.c_str(),imgL,vImgL,count);

//            imageListR.push_back(imr);
//            imageListL.push_back(iml);
//            imageListLR.push_back(iml);
//            imageListLR.push_back(imr);
            cv::Mat cL(pointbufL);
            cv::Mat cR(pointbufR);
            drawChessboardCorners(mat, boardSize, cL, foundL);
            drawChessboardCorners(vMat, boardSize, cR, foundR);
            cv::Mat h = cv::findHomography(pointbufL, pointbufR, CV_RANSAC);
            for (int r = 0; r < h.rows; ++r) {
                for (int c  = 0; c  < h.cols; ++c ) {
                    std::cout << h.at(r,c) << " ";
                }
                std::cout << std::endl;
            }
            
//            count++;
        }
        
    }
    
    return true;
    if (imageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr > &frame = imagePortOut.prepare();
        frame = imageCollector.getImage();
//        std::cout << "frame.height() = " << frame.height() << std::endl;
//        std::cout << "frame.width() = " << frame.width() << std::endl;
        ev::vQueue vQueue = eventCollector.getEvents();
//        std::cout << vQueue.size() << std::endl;
        for (auto it = vQueue.begin(); it != vQueue.end(); ++it){
            
            auto v = ev::is_event<ev::AE >(*it);
            if (v->channel) continue; //Skipping events from right cam
            double x = (303 - v->x);
            double y = (239 - v->y);
            yarp::sig::Vector evCoord (3);
            evCoord[0] = x;
            evCoord[1] = y;
            evCoord[2] = 1;
//            std::cout << "x bef = " << x << std::endl;
//            std::cout << "y bef = " << y << std::endl;
            
//            evCoord *= homography;
            
//            std::cout << "evCoord[0] = " << evCoord[0] << std::endl;
//            std::cout << "evCoord[1] = " << evCoord[1] << std::endl;
//            std::cout << "evCoord[2] = " << evCoord[2] << std::endl;
            //x *= 4.21;
            //y *= 4.26;
            x = evCoord[0] / evCoord[2];
            y = evCoord[1] / evCoord[2];
//            std::cout << "x = " << x << std::endl;
//            std::cout << "y = " << y << std::endl;
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
    setName(moduleName.c_str());
    homography.resize(3,3);
    homography(0,0) = 5.61627318;
    homography(0,1) = -9.53359703e-01;
    homography(0,2) = -4.22708305e+02;
    homography(1,0) = 1.46739935e-01;
    homography(1,1) =  5.14009647;
    homography(1,2) = -1.40622260e+02;
    homography(2,0) = -2.84753637e-04;
    homography(2,1) = -4.05376215e-04;
    homography(2,2) = 1;
    homography = homography.transposed();
//    homography = yarp::sig::Matrix::eye();
    for (int r = 0; r < homography.rows(); ++r) {
        for (int c = 0; c < homography.cols(); ++c) {
            std::cout << homography(r, c) << " ";
        }
        std::cout << std::endl;
    }
    imageCollector.open(getName("/img:i"));
    vImageCollector.open(getName("/vImg:i"));
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
