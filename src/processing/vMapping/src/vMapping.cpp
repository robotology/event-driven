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
    rf.setDefaultConfigFile( "vFramer.ini" );
    rf.configure( argc, argv );
    
    vMappingModule mappingModule;
    return mappingModule.runModule(rf);
}

bool vMappingModule::updateModule() {
    return true; //TODO
}

bool vMappingModule::configure( yarp::os::ResourceFinder &rf ) {
    return true;//TODO
}

bool vMappingModule::interruptModule() {
    return true;//TODO
}

bool vMappingModule::close() {
    return true;//TODO
}

double vMappingModule::getPeriod() {
    return 0;//TODO
}
