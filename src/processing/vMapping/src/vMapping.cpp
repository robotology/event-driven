//
// Created by miacono on 11/07/17.
//

#include <vMapping.h>


using namespace ev;

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
   
    if (imageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr > &frame = imagePortOut.prepare();
        frame = imageCollector.getImage();

        ev::vQueue vQueue = eventCollector.getEvents();
        //std::cout << vQueue.size() << std::endl;
        for (auto it = vQueue.begin(); it != vQueue.end(); ++it){

            auto v = ev::is_event<ev::AE >(*it);
            if (v->channel) continue;
            unsigned int x = (304 - v->x);
            unsigned int y = (240 - v->y);
            //x *= 4.21;
            //y *= 4.26;
            frame(x,y) = yarp::sig::PixelBgr(255,255,255);
        }
          imagePortOut.write();
    }
    
    return true;
}

bool vMappingModule::configure( yarp::os::ResourceFinder &rf ) {
    std::string moduleName = rf.check("name",yarp::os::Value("/vMapping")).asString();
    setName(moduleName.c_str());
    
    imageCollector.open(getName("/img:i"));
    eventCollector.open(getName("/vBottle:i"));
    imagePortOut.open(getName("/img:o"));
    eventCollector.start();
    imageCollector.start();
    return true;
}

bool vMappingModule::interruptModule() {
    imagePortOut.interrupt();
    imageCollector.interrupt();
    eventCollector.interrupt();
    return true;
}

bool vMappingModule::close() {
    imagePortOut.close();
    imageCollector.close();
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
