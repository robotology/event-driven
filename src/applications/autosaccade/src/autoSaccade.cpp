/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Arren.Glover@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "autoSaccade.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* create the module */
    AutoSaccadeModule saccadeModuleInstance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "autosaccade.ini" );
    rf.configure( argc, argv );

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return saccadeModuleInstance.runModule(rf);

}

/***********************SaccadeModule***********************/

bool AutoSaccadeModule::configure(ResourceFinder &rf){
    readParams( rf );
    bool configDone = true;
    configDone &= openJointControlDriver();
    configDone &= openGazeDriver();
    configDone &= openPorts();
    
    //initialize timestamp
    prevStamp =  0;
    
    return configDone;
}

bool AutoSaccadeModule::openPorts() {
    bool check = true;
    check &= eventBottleManager.open( getName( "/vBottle:i" ) );
    check &= vRatePort.open( getName( "/vRate:o" ) );
    //check &= leftImgPort.open( getName( "/imgL:o" ) );
    //check &= rightImagePort.open( getName( "/imgR:o" ) );
    check &= rpcPort.open( getName( "/rpc" ) );
    
    if (check) {
        attach(rpcPort);
        return true;
    } else {
        cerr << "Could not open some port" << endl;
        return false;
    }
}

void AutoSaccadeModule::readParams( const ResourceFinder &rf ) {//set the name of the module
    string moduleName = rf.check("name", Value("autoSaccade")).asString();
    robotName = rf.check("robotName", Value("icubSim")).asString();
    //Append slash at beginning of moduleName to comply with port naming convention
    if (moduleName[0] != '/'){
        moduleName = '/' + moduleName;
    }
    if (robotName[0] != '/'){
        robotName = '/' + robotName;
    }
    setName( moduleName.c_str() );
    //Read parameters
    checkPeriod = rf.check( "checkPeriod", Value( 0.1 ) ).asDouble();
    minVpS = rf.check( "minVpS", Value( 75000 ) ).asDouble();
    timeout = rf.check( "timeout", Value( 1.0 ) ).asDouble();
    refSpeed = rf.check( "refSpeed", Value( 300.0 ) ).asDouble();
    refAcc = rf.check( "refAcc", Value( 200.0 ) ).asDouble();
    camWidth = rf.check( "camWidth", Value( 304 ) ).asInt();
    camHeight = rf.check( "camHeight", Value( 240 ) ).asInt();
    
}

bool AutoSaccadeModule::openGazeDriver() {//open driver for gaze control
    Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", getName( "/gazeCtrl" ) );
    options.put("remote","/iKinGazeCtrl");
    gazeDriver.open( options );
    if(!gazeDriver.isValid()) {
        cerr << "Did not connect to robot/simulator" << endl;
        return false;
    }
    else {
        gazeDriver.view( gazeControl );
    }
    
    if(!gazeControl ) {
        cerr << "Did not connect to gaze controller" << endl;
        return false;
    }
    gazeControl->storeContext( &context0 );
    return true;
}

bool AutoSaccadeModule::openJointControlDriver() {//open driver for joint control
    Property options;
    options.put("device","remote_controlboard");
    options.put("remote",robotName + "/head");
    options.put("local", getName( "/head" ) );
    
    mdriver.open( options );
    if(!mdriver.isValid()) {
        cerr << "Did not connect to robot/simulator" << endl;
        return false;
    } else {
        mdriver.view( ipos );
        mdriver.view( imod );
    }
    if (!ipos || !imod){
        cerr << "Could not open joint control driver" << endl;
        return false;
    }
    
    bool check = true;
    for ( int i = 0; i <= 5; ++i ) {
        check &= configDriver( i, 30.0, 200.0 );
    }
    return check;
}

bool AutoSaccadeModule::configDriver( int joint, double refSp, double refAcc ) {
    if ( ipos && imod ) {
        ipos->setRefSpeed( joint, refSp );
        ipos->setRefAcceleration( joint, refAcc );
        imod->setControlMode( joint, VOCAB_CM_POSITION );
    } else {
        cerr << "Could not open driver" << endl;
        return false;
    }
    return true;
}

bool AutoSaccadeModule::interruptModule() {
    cout << "Interrupting" << endl;
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    cout << "Finished Interrupting" << endl;
    return true;
}

bool AutoSaccadeModule::close() {
    
    cout << "Closing" << endl;
    rpcPort.close();
    eventBottleManager.close();
    mdriver.close();
    gazeDriver.close();
    //delete ipos; delete imod;
    cout << "Finished Closing" << endl;
    return true;
}

void AutoSaccadeModule::performSaccade() {
    for ( double theta = 0; theta < 2*M_PI; theta+= M_PI/36 ) {
        ipos->positionMove( 3, cos( theta ) );
        ipos->positionMove( 4, sin( theta ) );
        Time::delay(0.005);
    }
    bool motionDone;
    int joints[2] = {3,4};
    while (!motionDone){
        ipos ->checkMotionDone(2,joints, &motionDone);
    }
    Time::delay(0.2);

}

double AutoSaccadeModule::computeEventRate() {
    //compute event rate
    double latestStamp = eventBottleManager.getTime();
    double vPeriod = latestStamp - prevStamp;
    
    if(vPeriod <= 0)
        return 0;
    
    vPeriod *= 80 *10e-9;
    double vCount = eventBottleManager.popCount();
    const double eventRate = vCount / vPeriod;
    prevStamp = latestStamp;
    return eventRate;
}

bool AutoSaccadeModule::updateModule() {


    
    //collect events for some time
    eventBottleManager.start();
    Time::delay(timeout);
    eventBottleManager.stop();

    //if there is no connection don't do anything yet
    if(!eventBottleManager.getInputCount()) return true;
    
    double eventRate = eventBottleManager.getEventRate();
    std::cout << "Event Rate: " << eventRate << std::endl;
    
    //output the event rate for debug purposes
    Bottle vRateBottle;
    vRateBottle.addDouble( eventRate );
    vRatePort.write(vRateBottle);
    
    //ImageOf<PixelBgr> &leftImage = leftImgPort.prepare();
    //ImageOf<PixelBgr> &rightImage = rightImagePort.prepare();

    //visualizeEvents( leftImage, rightImage, q );
    
    //Face straight (for simulation only)
    if (robotName == "/icubSim")
        home();
    
    //if event rate is low then saccade, else gaze to center of mass of events
    if(eventRate < minVpS) {
        cout << "perform saccade " << endl;
        
        //Stop gaze and reconfig driver to restore joint control mode
        gazeControl->stopControl();
        
        configDriver( 3, refSpeed, refAcc );
        configDriver( 4, refSpeed, refAcc );
        
        performSaccade();
    } else {

        ev::vQueue q = eventBottleManager.getEvents();

        Vector cmL,cmR;
        
        if (computeCenterMass( cmR, cmL, q )) {
            cout << "gazing at l:(" << cmL( 0 ) << ", " << cmL( 1 ) << ")" << endl;
            cout << "          r:(" << cmR( 0 ) << ", " << cmR( 1 ) << ")" << endl;
    
            //Making attention point red in image
            //leftImage((int) cmL( 0 ),(int) cmL( 1 ) ) = PixelBgr( 255, 0, 0 );
            //rightImage( (int) cmR( 0 ),(int) cmR( 1 ) ) = PixelBgr( 255, 0, 0 );

            //Images are flipped wrt camera orientation
            cmL(0) = camWidth - 1 - cmL(0);
            cmL(1) = camHeight - 1 - cmL(1);
            
            cmR(0) = camWidth - 1 - cmR(0);
            cmR(1) = camHeight - 1 - cmR(1);

            gazeControl->restoreContext( context0 );
            gazeControl->lookAtMonoPixelSync(0, cmL);
            //gazeControl->lookAtStereoPixels( cmL, cmR );
            gazeControl->waitMotionDone( 0.1, 4.0 );
            cout << "Finished gazing" << endl;
        }
    }

    //leftImgPort.write();
    //rightImagePort.write();
    return true;
}

/*
void AutoSaccadeModule::visualizeEvents( ImageOf<PixelBgr> &leftImage, ImageOf<PixelBgr> &rightImage, ev::vQueue &q ) const {
    vFeatureMap lMap( 240, 304 );
    vFeatureMap rMap(240,304);
    
    for ( ev::vQueue::iterator i = q.begin(); i != q.end(); ++i ) {
        auto aep = ev::is_event<ev::AE>( *i );
        if (aep.get()->channel) {
            rMap(aep.get()->y, aep.get()->x) += 80;
        } else {
            lMap (aep.get()->y, aep.get()->x) += 80;
        }
    }
    lMap.convertToImage(leftImage);
    rMap.convertToImage(rightImage);
}
*/
void AutoSaccadeModule::home() {
    gazeControl->stopControl();
    double homePos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    for ( int i = 0; i <= 5; ++i ) {
        configDriver( i, 30.0, 200.0 );
    }
    
    ipos->positionMove(homePos);
    
    bool motionDone = false;
    
    while (!motionDone){
        ipos->checkMotionDone(&motionDone);
    }
    
    Time::delay(1.0);
}

bool AutoSaccadeModule::computeCenterMass( Vector &cmR, Vector &cmL, ev::vQueue &q ) {
    
    if (q.empty()) {
        cerr << "Could not compute center of mass: empty event queue" << endl;
        return false;
    }
    
    int xl = 0, yl = 0;
    int xr = 0, yr = 0;
    int rSize = 0, lSize = 0;
    cmR.resize(2);
    cmL.resize(2);
    for ( ev::vQueue::iterator i = q.begin(); i != q.end(); ++i ) {
        auto aep = ev::is_event<ev::AE>( *i );
        if (aep->channel) {
            xr += aep->x;
            yr += aep->y;
            rSize++;
        } else {
            xl += aep->x;
            yl += aep->y;
            lSize++;
        }
    }
    
    if (lSize == 0 || rSize == 0)
        return false;
    
    xl /= lSize;
    yl /= lSize;
    xr /= rSize;
    yr /= rSize;
    
    cmR(0) = xr;
    cmR(1) = yr;
    cmL(0) = xl;
    cmL(1) = yl;
    
    return true;
}

double AutoSaccadeModule::getPeriod() {
    return checkPeriod;
}

bool AutoSaccadeModule::respond(const Bottle &command, Bottle &reply) {
    //fill in all command/response plus module update methods here
    return true;
}

/***********************EventBottleManager***********************/

EventBottleManager::EventBottleManager() {
    
    //here we should initialise the module
    vCount = 0;
    latestStamp = 0;
    isReading = false;
}

bool EventBottleManager::open(const string &name) {
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
        return;
    }
    
    mutex.wait();
    //append new events to queue
    vQueue.insert(vQueue.end(), newQueue.begin(), newQueue.end());
    latestStamp = unwrapper(newQueue.back()->stamp);
    vCount += newQueue.size();
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
    //if (!&vQueue)
    //    return ev::vQueue();
    //ev::vQueue outQueue = vQueue;
    //vQueue.clear();
    return vQueue;
}

//empty line to make gcc happy
