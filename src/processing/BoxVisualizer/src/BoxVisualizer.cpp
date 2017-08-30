//
// Created by miacono on 11/08/17.
//

#include <algorithm>
#include <BoxVisualizer.h>
using namespace yarp::math;

void clamp (double &val, double min, double max){
    val = std::min (max, val);
    val = std::max (min, val);
}


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
    rf.setDefaultConfigFile( "DualCamTransform.ini" );
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
    createImg = rf.check("createImg",yarp::os::Value(false)).asBool();
    
    this -> confFileName = rf.getHomeContextPath().c_str();
    confFileName += "/DualCamTransform.ini";
    
    height = rf.check("height", yarp::os::Value(240)).asInt();
    width = rf.check("width", yarp::os::Value(304)).asInt();
    channel = rf.check("channel", yarp::os::Value(0)).asInt();
    leftH.resize(3,3);
    bool ok = readConfigFile( rf, "MAPPING_LEFT", leftH );
    leftH = yarp::math::luinv(leftH);
    ok &= vPortIn.open(getName("/vBottle:i"));
    ok &= vPortOut.open(getName("/vBottle:o"));
    vPortIn.startReadingLeft();
    ok &= boxesPortIn.open(getName("/boxes:i"));
    ok &= boxesPortOut.open(getName("/boxes:o"));
    if (createImg)
        ok &= imgPortOut.open(getName("/img:o"));
    return ok;
}

bool BoxVisualizer::interruptModule() {
    vPortIn.interrupt();
    vPortOut.interrupt();
    boxesPortIn.interrupt();
    boxesPortOut.interrupt();
    imgPortOut.interrupt();
    return true;
}

bool BoxVisualizer::close() {
    vPortIn.close();
    vPortOut.close();
    boxesPortIn.close();
    boxesPortOut.close();
    imgPortOut.close();
    return true;
}

bool BoxVisualizer::updateModule() {
    
    
    if (boxesPortIn.isBoxReady()){
        yarp::os::Bottle boxBottle = boxesPortIn.getBox();
        std::vector<yarp::sig::Vector> corners(4);
        minY = boxBottle.get(0).asDouble();
        minX = boxBottle.get(1).asDouble();
        maxY = boxBottle.get(2).asDouble();
        maxX = boxBottle.get(3).asDouble();
        
        transformPoint( minX, minY, leftH );
        transformPoint( maxX, maxY, leftH );
        
        clamp(minY, 0 ,height - 1);
        clamp(maxY, 0 ,height - 1);
        clamp(minX, 0 ,width - 1);
        clamp(maxX, 0 ,width - 1);
        
        //Sending out transformed box
        yarp::os::Bottle &boxBottleOut = boxesPortOut.prepare();
        boxBottleOut.clear();
        boxBottleOut.addInt(minY);
        boxBottleOut.addInt(minX);
        boxBottleOut.addInt(maxY);
        boxBottleOut.addInt(maxX);
        boxesPortOut.write();
    }
    
    
    
    if (vPortIn.isPortReadingLeft() && vPortIn.hasNewEvents()) {
        ev::vQueue q = vPortIn.getEventsFromChannel( channel );
        ev::vBottle &vBottleOut = vPortOut.prepare();
        vBottleOut.clear();
        for ( auto &it : q ) {
            
            auto v = ev::is_event<ev::AE>( it );
            
            double x = v->x;
            double y = v->y;
            
            if ( x >= 0 && x < width && y >= 0 && y < height ) {
                if ( x >= minX && x <= maxX && y >= minY && y <= maxY ) {
//                    v->x -= minX;
//                    v->y -= minY;
                    vBottleOut.addEvent( v );
                }
            }
        }
        if ( createImg ) {
            createImage( q );
        }
        if (vBottleOut.size())
            vPortOut.write();
    }
    return true;
}

void BoxVisualizer::createImage( const ev::vQueue &q ) {
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imgOut = imgPortOut.prepare();
    imgOut.resize( width, height );
    imgOut.zero();
    for ( auto &it : q ) {
        
        auto v = ev::is_event<ev::AE>( it );
        
        double x = v->x;
        double y = v->y;
        
        if ( x >= 0 && x < width && y >= 0 && y < height ) {
            imgOut( x, y ) = yarp::sig::PixelBgr( 255, 255, 255 );
            if ( imgPortOut.getOutputCount() )
                imgPortOut.write();
        }
        drawRectangle( minY, minX, maxY, maxX, imgOut );
    }
}

void BoxVisualizer::drawRectangle( int minY, int minX, int maxY, int maxX
                                   , yarp::sig::ImageOf<yarp::sig::PixelBgr> &image )  {
    
    for ( int i = minX; i <= maxX; ++i ) {
        if(i >= 0 && i < image.width()) {
            image( i, minY ) = yarp::sig::PixelBgr( 255, 0, 0 );
            image( i, maxY ) = yarp::sig::PixelBgr( 255, 0, 0 );
        }
    }
    
    for ( int i = minY; i <= maxY; ++i ) {
        if (i >= 0 && i < image.height()) {
            image( minX, i ) = yarp::sig::PixelBgr( 255, 0, 0 );
            image( maxX, i ) = yarp::sig::PixelBgr( 255, 0, 0 );
        }
    }
}

void BoxVisualizer::transformPoint( double &x, double &y, yarp::sig::Matrix homography ) const {
    yarp::sig::Vector point( 3 );
    
    //Converting to homogeneous coordinates
    point[0] = x;
    point[1] = y;
    point[2] = 1;
    
    //Applying trasformation
    point *= homography;
    
    //Converting back from homogenous coordinates
    x = point[0] / point[2];
    y = point[1] / point[2];
}

double BoxVisualizer::getPeriod() {
    return 0.01;
}

/***********************EventPort***********************/

void EventPort::onRead(ev::vBottle &bot) {
    if (!isReadingLeft && ! isReadingRight)
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
        if (v->channel) {
            if ( isReadingRight ) {
                vRightQueue.push_back( v );
                newEventsRight = true;
                
            }
        }else {
            if ( isReadingLeft ) {
                vLeftQueue.push_back( v );
                newEventsLeft = true;
            }
        }
    }
    mutex.post();
}

ev::vQueue EventPort::getEventsFromChannel( int channel ) {
    
    ev::vQueue outQueue;
    mutex.wait();
    if (channel){
        outQueue = vRightQueue;
        vRightQueue.clear();
        newEventsRight = false;
    } else {
        outQueue = vLeftQueue;
        vLeftQueue.clear();
        newEventsLeft = false;
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

/***********************BoxesPort***********************/

void BoxesPort::onRead( yarp::os::Bottle &bot ) {
    boxBottle = bot;
    ready = true;
}
