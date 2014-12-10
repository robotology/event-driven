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

#include <iCub/HeadingModule.h>


 bool HeadingModule::configure(ResourceFinder & rf){

        string moduleName;
        string inPortName;
        string outPortName;

        int threadsNum;
        unsigned long smoothIntv;
        unsigned long frameIntv;
        int ttcRange;




        moduleName =  rf.check("name", Value("Heading"), "module name (String)").asString();
        setName(moduleName.c_str());


        threadsNum = rf.check("threadNum",
                               Value(4),
                               "number of threads (int)").asInt();

        frameIntv = rf.check("frame_intv",
                             Value(100000),
                             "time interval for each frame (unsinged long)").asInt();

        smoothIntv = rf.check("smooth_intv",
                             Value(20000),
                             "time interval for spatial and temporal smoothing (unsinged long)").asInt();

        ttcRange = rf.check("TTC_Range",
                              Value(15),
                              "the maximume value of TTC (int)").asInt();


        receptiveField = new SuperReceptiveField(threadsNum, frameIntv, smoothIntv, ttcRange);


        //open input BufferedPort
        inPortName = "/";
        inPortName += getName();
        inPortName += rf.check("inPortName",
                                    Value("/vels:i"),
                                    "Input  flow port (string)").asString();


        //open output Port
        outPortName = "/";
        outPortName += getName();
        outPortName += rf.check("outPortName",
                                    Value("/FOEMap:o"),
                                    "Output heading port (image)").asString();

        //Start the threads
        receptiveField->setPortNames(inPortName, outPortName);

        if (!receptiveField->start()){
            cerr  << "Heading Module: Sorry! Unable to start threads" << endl;
            return false;
        }

        return true;
    }


bool HeadingModule::updateModule(){

    return true;
}

bool HeadingModule::interruptModule(){
    cout << "Interrupting.." << endl;
    receptiveField->stop();
    return true;
}

bool HeadingModule::close(){

    cout << "closing .." << endl;
    return true;
}



HeadingModule::~HeadingModule(){
    delete receptiveField;
    cout << "Heading Module destructor, ended. " << endl;
}

