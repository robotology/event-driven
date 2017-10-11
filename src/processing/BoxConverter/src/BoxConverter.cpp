//
// Created by miacono on 11/08/17.
//

#include <algorithm>
#include <BoxConverter.h>
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
    
    BoxConverter boxVisualizerModule;
    return boxVisualizerModule.runModule(rf);
}

bool BoxConverter::readConfigFile( const yarp::os::ResourceFinder &rf, std::string groupName
                                   , yarp::sig::Matrix &homography ) const {
    yarp::os::Bottle &conf = rf.findGroup( groupName );
    
    //If config file not found, calibration necessary
    if ( conf.isNull() ) {
        yInfo() << "Could not find transform config in group " << groupName << ". Calibration is necessary.";
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

bool BoxConverter::configure( yarp::os::ResourceFinder &rf ) {
    
    //Reading command-line arguments
    std::string moduleName = rf.check("name",yarp::os::Value("/BoxConverter")).asString();
    setName(moduleName.c_str());
    bool invert = rf.check("invert",yarp::os::Value(true)).asBool();
    bool outputEvents = rf.check("outputEvents",yarp::os::Value(false)).asBool();
    height = rf.check("height", yarp::os::Value(240)).asInt();
    width = rf.check("width", yarp::os::Value(304)).asInt();
    channel = rf.check("channel", yarp::os::Value(0)).asInt();
    std::string groupName = rf.check("groupName",yarp::os::Value("TRANSFORM_LEFT")).asString();
    //Reading config file
    this -> confFileName = rf.getHomeContextPath().c_str();
    confFileName += "/DualCamTransform.ini";
    
    H.resize(3,3);
    bool ok = readConfigFile( rf, groupName, H );
    if (invert) {
        H = yarp::math::luinv( H );
    }
    
    //Opening required ports
    ok &= boxesPortIn.open(getName("/boxes:i"));
    ok &= vBoxesPortOut.open(getName("/boxes:o"));
    if (outputEvents) {
        ok &= vPortIn.open( getName( "/vBottle:i" ) );
        ok &= vPortOut.open( getName( "/vBottle:o" ) );
        if (channel) {
            vPortIn.startReadingLeft();
        } else {
            vPortIn.startReadingRight();
        }
    }
    
    return ok;
}

bool BoxConverter::interruptModule() {
    vPortIn.interrupt();
    vPortOut.interrupt();
    boxesPortIn.interrupt();
    vBoxesPortOut.interrupt();
    imgPortOut.interrupt();
    return true;
}

bool BoxConverter::close() {
    vPortIn.close();
    vPortOut.close();
    boxesPortIn.close();
    vBoxesPortOut.close();
    imgPortOut.close();
    return true;
}

bool BoxConverter::updateModule() {
    
    if (vPortIn.isPortReading() && vPortIn.hasNewEvents()) {
        ev::vQueue q = vPortIn.getEventsFromChannel( channel );
        ev::vBottle &vBottleOut = vPortOut.prepare();
        vBottleOut.clear();
        for ( auto &it : q ) {
            
            auto v = ev::is_event<ev::AE>( it );
            
            double x = v->x;
            double y = v->y;
            
            if ( x >= 0 && x < width && y >= 0 && y < height ) {
                if ( x >= minX && x <= maxX && y >= minY && y <= maxY ) {
                    vBottleOut.addEvent( v );
                }
            }
        }
        if (vBottleOut.size())
            vPortOut.write();
    }
    
    
    if (boxesPortIn.isBoxReady()){
        yarp::os::Bottle boxBottle = boxesPortIn.getBox();
        std::vector<yarp::sig::Vector> corners(4);
        minY = boxBottle.get(0).asDouble();
        minX = boxBottle.get(1).asDouble();
        maxY = boxBottle.get(2).asDouble();
        maxX = boxBottle.get(3).asDouble();
        
        transformPoint( minX, minY, H );
        transformPoint( maxX, maxY, H );
        
        clamp(minY, 0 ,height - 1);
        clamp(maxY, 0 ,height - 1);
        clamp(minX, 0 ,width - 1);
        clamp(maxX, 0 ,width - 1);
        
        //Sending out transformed box
        ev::vBottle &vBoxBottle = vBoxesPortOut.prepare();
        vBoxBottle.clear();
        auto vBox = ev::make_event<ev::BoxEvent>();
        vBox->x = minX;
        vBox->y = minY;
        vBox->width = maxX - minX;
        vBox->height = maxY - minY;
        vBoxBottle.addEvent(vBox);
        vBoxesPortOut.write(true);
    }
    return true;
}

void BoxConverter::transformPoint( double &x, double &y, yarp::sig::Matrix homography ) const {
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

double BoxConverter::getPeriod() {
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
