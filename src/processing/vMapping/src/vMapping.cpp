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
    if (calibrateLeft) {
        
        if ( leftImageCollector.isImageReady() && vLeftImageCollector.isImageReady() ) {
            yarp::sig::ImageOf<yarp::sig::PixelBgr> leftImg = leftImageCollector.getImage();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> vLeftImg = vLeftImageCollector.getImage();
            
            if (performCalibStep( leftImg, vLeftImg, leftH )) {
                nIter++;
                yInfo() << nIter << " of " << maxIter << " images collected";
            }
            //When max number of iteration is reached calibration is finalized
            if (nIter >= maxIter){
                finalizeCalibration( leftH, "MAPPING_LEFT" );
                calibrateLeft = false;
            }
        }
        return true;
    }
    
    if (calibrateRight) {
        
        if ( rightImageCollector.isImageReady() && vRightImageCollector.isImageReady() ) {
            yarp::sig::ImageOf<yarp::sig::PixelBgr> rightImg = rightImageCollector.getImage();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> vRightImg = vRightImageCollector.getImage();
            
            if (performCalibStep( rightImg, vRightImg, rightH )) {
                nIter++;
                yInfo() << nIter << " of " << maxIter << " images collected";
            }
            //When max number of iteration is reached calibration is finalized
            if (nIter >= maxIter){
                finalizeCalibration( rightH, "MAPPING_RIGHT" );
                calibrateRight = false;
            }
        }
        return true;
    }
    
    //If image is ready remap and draw events on it
    if (leftImageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr > &leftImg = leftImagePortOut.prepare();
        leftImg = leftImageCollector.getImage();
        ev::vQueue vLeftQueue = eventCollector.getEventsFromChannel(0);
        performMapping( leftImg, vLeftQueue, leftH );
        leftImagePortOut.write();
    }
    
    if (rightImageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr > &rightImg = rightImagePortOut.prepare();
        rightImg = rightImageCollector.getImage();
        ev::vQueue vRightQueue = eventCollector.getEventsFromChannel(1);
        performMapping( rightImg, vRightQueue, rightH );
        rightImagePortOut.write();
    }
    
    return true;
    
    
}

void vMappingModule::performMapping( yarp::sig::ImageOf<yarp::sig::PixelBgr> &img, const vQueue &vQueue
                                     , const yarp::sig::Matrix &homography ) const {
    for ( auto &it : vQueue ) {
        
        auto v = is_event<AE >( it );
        
        double x = (303 - v->x);
        double y = (239 - v->y);
        yarp::sig::Vector evCoord( 3 );
        
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
        if (x >= 0 && x < img.width())
            if (y >= 0 && y < img.height())
                img( x, y ) = yarp::sig::PixelBgr( 255, 255, 255 );
    }
}

void vMappingModule::finalizeCalibration( yarp::sig::Matrix &homography, std::string groupName ) {
    homography /= nIter;
    homography = homography.transposed();
    cv::destroyAllWindows();
    yInfo() << "Camera calibration is over after " << nIter << " steps";
    nIter = 0;
    yInfo() << "Saving calibration results to " << confFileName;
    std::fstream confFile;
    confFile.open( confFileName.c_str() );
    std::string line;
    bool groupFound = false;
    if (confFile.is_open()){
        while (std::getline(confFile, line)){
            if (line.find("[" + groupName +"]") != std::string::npos) {
                groupFound = true;
                break;
            }
        }
        if (!groupFound) {
            confFile.close();
            confFile.open(confFileName, std::ios::app);
            if (confFile.is_open())
                confFile << "\n[" << groupName << "]" << std::endl;
        }
        confFile << "homography ( ";
        for (int r = 0; r < homography.rows(); ++r) {
            for (int c  = 0; c  < homography.cols(); ++c ) {
                confFile << homography(r, c ) << " ";
            }
        }
        confFile << ") \n";
        confFile.close();
    } else {
        yError() << "Cannot open config file, results not saved";
    }
}

bool vMappingModule::performCalibStep( yarp::sig::ImageOf<yarp::sig::PixelBgr> &frame
                                       , yarp::sig::ImageOf<yarp::sig::PixelBgr> &vImg, yarp::sig::Matrix &homography ) const {
    auto *frameIplImg = (IplImage *) frame.getIplImage();
    cv::Mat frameMat = cv::cvarrToMat( frameIplImg );
    imshow( "img", frameMat );
    
    auto *vIplImg = (IplImage *) vImg.getIplImage();
    cv::Mat vMat = cv::cvarrToMat( vIplImg );
    imshow( "vImg", vMat );
    cv::waitKey( 1 );
    cv::Size boardSize( 4, 11 );
    std::vector<cv::Point2f> frameCenters; //Vector for storing centers of circle grid on frame image
    std::vector<cv::Point2f> vCenters; //Vector for storing centers of circle grid on event image
    
    bool frameFound = findCirclesGrid( frameMat, boardSize, frameCenters,
                                       cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING );
    bool eventFound = findCirclesGrid( vMat, boardSize, vCenters,
                                       cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING );
    
    if ( frameFound && eventFound ) {
        cvCvtColor( frameIplImg, frameIplImg, CV_RGB2BGR );
        cvCvtColor( vIplImg, vIplImg, CV_RGB2BGR );
        
        cv::Mat frameCentersMat( frameCenters );
        cv::Mat vCentersMat( vCenters );
        drawChessboardCorners( frameMat, boardSize, frameCentersMat, frameFound );
        drawChessboardCorners( vMat, boardSize, vCentersMat, eventFound );
        cv::Mat h = findHomography( vCenters, frameCenters, CV_RANSAC );
        imshow( "img", frameMat );
        imshow( "vImg", vMat );
        cv::waitKey( 1000 );
        for ( int r = 0; r < homography.rows(); ++r) {
            for ( int c  = 0; c  < homography.cols(); ++c ) {
                homography( r, c ) += h.at<double>( r, c );
            }
        }
        return true;
    } else {
        return false;
    }
}

bool vMappingModule::configure( yarp::os::ResourceFinder &rf ) {
    std::string moduleName = rf.check("name",yarp::os::Value("/vMapping")).asString();
    setName(moduleName.c_str());
    
    calibrateLeft = rf.check("calibrateLeft", yarp::os::Value(false)).asBool();
    calibrateRight = rf.check("calibrateRight", yarp::os::Value(false)).asBool();
    
    this -> confFileName = rf.getHomeContextPath().c_str();
    confFileName += "/vMapping.ini";
    
    leftH.resize( 3, 3 );
    rightH.resize( 3, 3 );
    //If no calibration required, read homography from config file
    if (!calibrateLeft) {
        calibrateLeft = !readConfigFile( rf, "MAPPING_LEFT", leftH ); //If no config found, calibration necessary
    }
    
    //If no calibration required, read homography from config file
    if (!calibrateRight) {
        calibrateRight = !readConfigFile( rf, "MAPPING_RIGHT", rightH ); //If no config found, calibration necessary
    }
    
    //Initialize calibration
    if (calibrateLeft) {
        vLeftImageCollector.open( getName( "/left/vImg:i" ) );
        maxIter = rf.check( "maxIter", yarp::os::Value( 20 ) ).asInt();
        nIter = 0;
        vLeftImageCollector.start();
    }
    
    if (calibrateRight) {
        vRightImageCollector.open( getName( "/right/vImg:i" ) );
        maxIter = rf.check( "maxIter", yarp::os::Value( 20 ) ).asInt();
        nIter = 0;
        vRightImageCollector.start();
    }
    
    leftImageCollector.open(getName("/left/img:i"));
    rightImageCollector.open(getName("/right/img:i"));
    eventCollector.open(getName("/vBottle:i"));
    leftImagePortOut.open(getName("/left/img:o"));
    rightImagePortOut.open(getName("/right/img:o"));
    eventCollector.start();
    leftImageCollector.start();
    rightImageCollector.start();
    return true;
}

bool vMappingModule::readConfigFile( const yarp::os::ResourceFinder &rf, std::string groupName
                                     , yarp::sig::Matrix &homography ) const {
    yarp::os::Bottle &conf = rf.findGroup( groupName );
    
    //If config file not found, calibration necessary
    if ( conf.isNull() ) {
        yInfo() << "Could not find mapping config in group " << groupName << ". Calibration is necessary.";
        return false;
    }
    
    yarp::os::Bottle *list = conf.find( "homography" ).asList();
    
    if ( list->size() != 9 ) {
        yError() << "Config file in " << groupName << "corrupted. Calibration is neccessary";
        return false;
    }
    
    for ( int r = 0; r < homography.rows(); ++r ) {
        for ( int c = 0; c < homography.cols(); ++c ) {
            homography( r, c ) = list->get( r * homography.rows() + c ).asDouble();
        }
    }
    
    return true;
}

bool vMappingModule::interruptModule() {
    leftImagePortOut.interrupt();
    leftImageCollector.interrupt();
    vLeftImageCollector.interrupt();
    eventCollector.interrupt();
    return true;
}

bool vMappingModule::close() {
    leftImagePortOut.close();
    leftImageCollector.close();
    vLeftImageCollector.close();
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

/***********************EventPort***********************/


bool EventPort::open(const std::string &name) {
    //and open the input port
    this->useCallback();
    return yarp::os::BufferedPort<ev::vBottle>::open(name);
}

void EventPort::onRead(ev::vBottle &bot) {
    //get new events
    ev::vQueue newQueue = bot.get<ev::AE>();
    if(newQueue.empty()){
        return;
    }
    
    mutex.wait();
    //append new events to queue

    for ( auto &it : newQueue ) {
        auto v = ev::is_event<ev::AE >( it );
        if (v->channel)
            vRightQueue.push_back(v);
        else
            vLeftQueue.push_back(v);
    }
    mutex.post();
}

ev::vQueue EventPort::getEventsFromChannel( int channel ) {

    ev::vQueue outQueue;
    mutex.wait();
    if (channel){
        outQueue = vRightQueue;
        vRightQueue.clear();
    } else {
        outQueue = vLeftQueue;
        vLeftQueue.clear();
    }
    mutex.post();
    return outQueue;
}
