//#include "opticalFlowViewer.h"
#include "flowViewer.h"
#include "VelocityBuffer.h"
#include <string>
#include <iostream>

#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Semaphore.h>

#include <queue>

using namespace std;
using namespace yarp::os;




class FlowViewerModule : public RFModule{
    flowViewer viewer;
    VelocityGrabber vGrabber;
    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono16> > outPort;
	yarp::os::BufferedPort< Bottle > outDataPort;


public:
    bool configure(ResourceFinder & rf){
        int visType;
        string moduleName;
        string inPortName;
        string outPortName;
        //unsigned long frameInv;
        double frameInv;
        double camClockFreq;

        moduleName =  rf.check("name", Value("OptFlViewer"), "module name (String)").asString();
        setName(moduleName.c_str());

        visType = rf.check("vis_method", Value(1), "visualization type").asInt();
        viewer.setVisMethod(visType);


        frameInv = rf.check("frame_intv",
                             Value(.030), //12500(2ms) 18750(3ms) 31250(5ms) 62500(10ms)
                             "time interval for each frame (unsinged long)").asDouble();

        camClockFreq = rf.check("cam_clk_freq",
                                     Value(.000001), //12500(2ms) 18750(3ms) 31250(5ms) 62500(10ms)
                                     "time interval for each frame (unsinged long)").asDouble();

        vGrabber.setParam(frameInv, camClockFreq);

        //open input BufferedPort
        inPortName = "/";
        inPortName += getName();
        inPortName += "/vels:i";
        if (!vGrabber.open(inPortName.c_str())){
            cerr << getName() << ": Sorry. Unable to open input port" << inPortName << endl;
            return false;
        }
        vGrabber.useCallback();

        //open image output Port
        outPortName = "/";
        outPortName += getName();
        outPortName += "/vels:o";
        if (!outPort.open(outPortName.c_str()) ){
            cerr << getName() << "" << outPortName << endl;
            return false;
        }

        //open data output Port
        outPortName = "/";
        outPortName += getName();
        outPortName += "/data:o";
        if (!outDataPort.open(outPortName.c_str()) ){
            cerr << getName() << "" << outPortName << endl;
            return false;
        }

        viewer.setPorts(&outPort, &vGrabber, &outDataPort);

        viewer.start();

        return true;
    }


    bool updateModule(){
        return true;

    }

    double getPeriod(){
        return 1;
    }

    bool interruptModule(){
        viewer.stop();
        cout << "Interrupting.." << endl;
        vGrabber.interrupt();
        outPort.interrupt();
		outDataPort.interrupt();
        return true;
    }

    bool close(){
        cout << "closing .." << endl;
        vGrabber.close();
        outPort.close();
		outDataPort.close();
        return true;
    }

    virtual ~FlowViewerModule(){
        cout << "Optical Flow Viewer Module is coles happily!" << endl;
    }
};


int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    FlowViewerModule fvModule;

    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultConfigFile("eventBasedOpticalFlow.ini"); //overriden by --from parameter
    //rf.setDefaultContext("eventOpticalFlow/conf"); //overriden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    if (!fvModule.configure(rf)){
        cerr << "Error in Configuring opticalFlow Viewer Module, returning" << endl;
        return -1;
    }

   fvModule.runModule();

   cout << "OpticalFlow Viewer Module shutting down." << endl;
    
   return 0;
}

