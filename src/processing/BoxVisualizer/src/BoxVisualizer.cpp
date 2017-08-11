//
// Created by miacono on 11/08/17.
//

#include <BoxVisualizer.h>
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
    
    BoxVisualizer mappingModule;
    return mappingModule.runModule(rf);
}

bool BoxVisualizer::readConfigFile( const yarp::os::ResourceFinder &rf, std::string groupName
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

bool BoxVisualizer::configure( yarp::os::ResourceFinder &rf ) {
    
    std::string moduleName = rf.check("name",yarp::os::Value("/BoxVisualizer")).asString();
    setName(moduleName.c_str());
    this -> confFileName = rf.getHomeContextPath().c_str();
    confFileName += "/vMapping.ini";
    
    height = rf.check("height", yarp::os::Value(240)).asInt();
    width = rf.check("width", yarp::os::Value(304)).asInt();
    channel = rf.check("channel", yarp::os::Value(0)).asInt();
    leftH.resize(3,3);
    bool ok = readConfigFile( rf, "MAPPING_LEFT", leftH );
    leftH = yarp::math::luinv(leftH);
    ok &= vPortIn.open(getName("/vBottle:i"));
    vPortIn.startReading();
    ok &= boxesPortIn.open(getName("/boxes:i"));
    ok &= imgPortOut.open(getName("/img:o"));
    return ok;
}

bool BoxVisualizer::interruptModule() {
    vPortIn.interrupt();
    boxesPortIn.interrupt();
    imgPortOut.interrupt();
    return true;
}

bool BoxVisualizer::close() {
    vPortIn.close();
    boxesPortIn.close();
    imgPortOut.close();
    return true;
}

bool BoxVisualizer::updateModule() {
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imgOut = imgPortOut.prepare();
    imgOut.resize(width, height);
    imgOut.zero();
    
    
    if (boxesPortIn.isBoxReady()){
        yarp::os::Bottle boxBottle = boxesPortIn.getBox();
        std::vector<yarp::sig::Vector> corners(4);
        double minY = boxBottle.get(0).asDouble() - 1;
        double minX = boxBottle.get(1).asDouble() - 1;
        double maxY = boxBottle.get(2).asDouble() - 1;
        double maxX = boxBottle.get(3).asDouble() - 1;
        
        transformPoint( minX, minY, leftH );
        transformPoint( maxX, maxY, leftH );
        
        drawRectangle( minY, minX, maxY, maxX, imgOut );
        
    }
    
    if (vPortIn.isPortReading()) {
        ev::vQueue q = vPortIn.getEventsFromChannel(channel);
        for (auto &it : q){
            
            auto v = ev::is_event<ev::AE >( it );
            
            double x = v->x;
            double y = v->y;
            if (x >= 0 && x < imgOut.width() && y >=0 && y < imgOut.height())
            imgOut( x, y ) = yarp::sig::PixelBgr( 255, 255, 255 );
        }
        imgPortOut.write();
    }
    return true;
}

void BoxVisualizer::drawRectangle( int minY, int minX, int maxY, int maxX
                                   , yarp::sig::ImageOf<yarp::sig::PixelBgr> &image )  {
    
    for ( int i = minX; i < maxX; ++i ) {
        if(i >= 0 && i < image.width()) {
            image( i, minY ) = yarp::sig::PixelBgr( 255, 0, 0 );
            image( i, maxY ) = yarp::sig::PixelBgr( 255, 0, 0 );
        }
    }
    
    for ( int i = minY; i < maxY; ++i ) {
        if (i >= 0 && i < image.height()) {
            image( minX, i ) = yarp::sig::PixelBgr( 255, 0, 0 );
            image( maxX, i ) = yarp::sig::PixelBgr( 255, 0, 0 );
        }
    }
}

void BoxVisualizer::transformPoint( double &x, double &y, yarp::sig::Matrix homography ) const {
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
}

double BoxVisualizer::getPeriod() {
    return 0.01;
}



/***********************EventPort***********************/


void EventPort::onRead(ev::vBottle &bot) {
    if (!isReading)
        return;
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

void EventPort::clearQueues() {
    mutex.wait();
    vLeftQueue.clear();
    vRightQueue.clear();
    mutex.post();
}



void BoxesPort::onRead( yarp::os::Bottle &bot ) {
        outBottle = bot;
        ready = true;
}
