/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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

#include "OpticalFlowModule.h"
#include <stdlib.h>

OpticalFlowModule::OpticalFlowModule(){
//	 wrldFlw = NULL;
	 inputPort = NULL;
	 evntsMutex = NULL;

}


bool OpticalFlowModule::configure(ResourceFinder & rf){
    string moduleName;
    string inPortName;
    string outFlwPortName;
    string outWrldMdlName;
    string outBttlPortName;

    short mtrxRowNo, mtrxClmnNo;

    moduleName =  rf.check("name", Value("eventBaseOpticalFlow"), "module name (String)").asString();
    setName(moduleName.c_str());
    //robotName = rf.check("robot", Value("icub"), "Robot name (string)").asString();

    cout << moduleName << endl;

    retinaSizeC = RETINA_SIZE_C;
    retinaSizeR = RETINA_SIZE_R;

    frameInv = rf.check("frame_intv",
    		            Value(18750), //12500(2ms) 18750(3ms) 31250(5ms) 62500(10ms)
    		            "time interval for each frame (unsinged long)").asInt();


    evntsMutex = new yarp::os::Semaphore(0);

    inputPort = new AERGrabber(evntsMutex, frameInv);
    //open BufferedPort
    inPortName = "/";
    inPortName += getName();
    inPortName += rf.check("inPortName",
                            Value("/events:i"),
                            "Input  event camera port (string)").asString();

//    if (!inFlowPort.open(inPortName.c_str())){
//        cerr << getName() << " : Sorry!! Unable to open input port to receive events from camera " << inPortName << endl;
//        return false;
//    }

    if (!inputPort->open(inPortName.c_str())){
        cerr << getName() << " : Sorry!! Unable to open input port to receive events from camera " << inPortName << endl;
        return false;
    }
    inputPort->useCallback();


    outFlwPortName = "/";
    outFlwPortName += getName();
    outFlwPortName += rf.check("outFlowPortName",
                            Value("/opticalFlow:o"),
                            "Output velocity port (VelocityBuffer)").asString();



    if (!outFlowPort.open(outFlwPortName.c_str())){
        cerr << getName() << ": Sorry!! Unable to open output port " << outFlwPortName << endl;
        return false;
    }

    outBttlPortName = "/";
    outBttlPortName += getName();
    outBttlPortName += rf.check("bottleFlowPort",
                            Value("/velocity:o"),
                            "Output velocity port(Bottle)").asString();
    if (!bottleFlowPort.open(outBttlPortName.c_str()) ){
        cerr << getName() << ": Sorry!! Unable to open output port " << outBttlPortName << endl;
        return false;
    }


    outWrldMdlName = "/";
    outWrldMdlName += getName();
    outWrldMdlName += rf.check("outWorldPortName",
                             Value("/worldView:o"),
                             "Output World View port (String)").asString();
    if (!outWrldMdlPort.open(outWrldMdlName.c_str())){
        cerr << getName() << ": Sorry!! Unable to open world output port" << outWrldMdlName << endl
                                    << "you will not be able to see the world as I see :( " << endl;
    }



    mtrxRowNo = retinaSizeR + 2 * SPATIAL_MARGINE_ADDUP;
    mtrxClmnNo = retinaSizeC + 2 * SPATIAL_MARGINE_ADDUP;


    worldStatus.resize(mtrxRowNo, mtrxClmnNo);
    prevWorldStatus.resize(mtrxRowNo,mtrxClmnNo);
    timestamps.resize(mtrxRowNo, mtrxClmnNo);
    worldStatus.initialize(0);
    prevWorldStatus.initialize(0);



    wrldFlw = new WorldOptFlow(inputPort, &outFlowPort, &bottleFlowPort,
                               &worldStatus, &prevWorldStatus, &timestamps, evntsMutex);
    wrldFlw -> start();

    cout << "module " << getName() << ": configured successfully." << endl;


    return true;
}

bool OpticalFlowModule::interruptModule(){

	cout << "calling Module interrupt ... " << endl;

    inputPort->interrupt();
    outFlowPort.interrupt();
    bottleFlowPort.interrupt();
    outWrldMdlPort.interrupt();

    cout << "Module Interrupt ended." << endl;
    return true;
}

bool OpticalFlowModule::close(){

	cout << " Module closing ...." << endl;

	wrldFlw->stop();

    inputPort->close();
    outFlowPort.close();
    bottleFlowPort.close();
    outWrldMdlPort.close();

	cout << "thread is stopped" << endl;
    cout << "Optical Flow Module: close function is called. " << endl;
    return true;
}

bool OpticalFlowModule::updateModule(){

   // if (evntsMutex->check()){
	//    wrldFlw->run();
	    worldStatusRenderer();
   // }
   return true;
}

double OpticalFlowModule::getPeriod(){
    return .01;
}

OpticalFlowModule::~OpticalFlowModule(){

	cout << "calling module destructor "<< endl;

    if (wrldFlw != NULL)
        delete wrldFlw;

    if ( inputPort != NULL)
    	delete inputPort;

    if (evntsMutex != NULL)
        delete evntsMutex;


    cout << "Optical flow Module destructor Finished." << endl;
}

void OpticalFlowModule::worldStatusRenderer(){

	if (outWrldMdlPort.getOutputCount()) {
		yarp::sig::ImageOf<yarp::sig::PixelMono16>& imgSnd=outWrldMdlPort.prepare();
		imgSnd.resize(128,128);
		 for (int i = 0; i < retinaSizeC; ++i) {
			for (int j = 0; j < retinaSizeR; ++j) {
				imgSnd (i,j) = worldStatus(j+SPATIAL_MARGINE_ADDUP,i+SPATIAL_MARGINE_ADDUP)*20+127;
			}
		 }
		 outWrldMdlPort.write();
	}

}
