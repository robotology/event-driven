/*
 * FOEModule.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: fuozhan
 */

#include <string>
#include <iostream>

#include <yarp/os/RFModule.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>

#include "VelocityBuffer.h"
#include "FOEFinder.h"
#include "flowClusters.h"
#include "ClusterViewer.h"


using namespace std;
using namespace yarp::os;

#define VEL_BFR_SIZE 10

class VelocityGrabber: public BufferedPort<VelocityBuffer>{

    unsigned int lostCntr;
    short firstIdx, lastIdx;
    yarp::os::Semaphore bfrMutex;
    VelocityBuffer * velBfr [VEL_BFR_SIZE];

public:
    VelocityGrabber(){
        lostCntr = 0;
        firstIdx = 0;
        lastIdx = 0;
    }

    virtual void onRead(VelocityBuffer & data){
        VelocityBuffer * vb = new VelocityBuffer();
        vb->setData(data);
        bfrMutex.wait();
        if (lastIdx < VEL_BFR_SIZE){
            velBfr[lastIdx++] = vb;
        }
        else{
            lostCntr ++;
            delete vb;
        }
        bfrMutex.post();
    }

    VelocityBuffer *  getVelocities(){
        VelocityBuffer * res = NULL;
        bfrMutex.wait();
        if (lastIdx - firstIdx > 0){
            res = velBfr[firstIdx++];
        }
        else {
            firstIdx = 0;
            lastIdx = 0;
        }
        bfrMutex.post();
        return res;

    }

    ~VelocityGrabber(){

        for (int i = firstIdx; i < lastIdx; ++i) {
            delete velBfr[i];
        }

        cout << "Sorry! " << lostCntr <<  " Velocity bufferes were lost." << endl;
    }
};



class FOEModule : public RFModule{
    flowClusters clusters;
    FOEFinder foeFinder;
    VelocityGrabber vGrabber;
    yarp::os::BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb> > outPort;
    unsigned long ts;

public:
    bool configure(ResourceFinder & rf){

        string moduleName;
        string inPortName;
        string outPortName;

        moduleName =  rf.check("name", Value("FOEFinder"), "module name (String)").asString();
        setName(moduleName.c_str());

        //open input BufferedPort
        inPortName = "/";
        inPortName += getName();
        inPortName += "/vels:i";
        if (!vGrabber.open(inPortName.c_str())){
            cerr << getName() << ": Sorry. Unable to open input port" << inPortName << endl;
            return false;
        }
        vGrabber.useCallback();

        //open output Port
        outPortName = "/";
        outPortName += getName();
        outPortName += "/FOEMap:o";
        if (!outPort.open(outPortName.c_str()) ){
            cerr << getName() << "" << outPortName << endl;
            return false;
        }

        foeFinder.setOutPort(&outPort);

        ts = 0;
        return true;
    }

    bool updateModule(){
        VelocityBuffer * vb;
        vb = vGrabber.getVelocities();
        ts ++;
        if (vb != NULL){
            foeFinder.bin2(*vb);
//       	  foeFinder.velNormal(*vb);
//            foeFinder.velDivergance(*vb);
           delete vb;
        }


        return true;

    }



    double getPeriod(){
        return .001;
    }

    bool interruptModule(){
        cout << "Interrupting.." << endl;
        vGrabber.interrupt();
        outPort.interrupt();
        return true;
    }

    bool close(){
        cout << "closing .." << endl;
        vGrabber.close();
        outPort.close();
        return true;
    }

    ~FOEModule(){
        cout << "Optical Flow Module is coles happily!" << endl;
    }
};



int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    FOEModule foeModule;

    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultConfigFile("eventBasedOpticalFlow.ini"); //overriden by --from parameter
    //rf.setDefaultContext("eventOpticalFlow/conf"); //overriden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    if (!foeModule.configure(rf)){
        cerr << "Error in Configuring opticalFlow Viewer Module, returning" << endl;
        return -1;
    }

   foeModule.runModule();

   cout << "OpticalFlow Viewer Module shutting down." << endl;

   return 0;
}
