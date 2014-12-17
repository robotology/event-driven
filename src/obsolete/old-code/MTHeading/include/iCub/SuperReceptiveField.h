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

#ifndef SUPERRECEPTIVEFIELD_H_
#define SUPERRECEPTIVEFIELD_H_

#include <vector>
#include <string>

#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Event.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/Matrix.h>

#include <yarp/sig/Image.h>

#include <iCub/VelocityBuffer.h>
//#include <iCub/emorph/VelocityBuffer.h>

#include "FOERecptiveField.h"
//#include "VelocityGrabber.h"

using namespace std;
using namespace yarp::os;

#define RETINA_X 128
#define RETINA_Y 128
#define FOEMAP_MAXREG_SIZE 7
#define TTCNGHBR_SIZE 7
//#define OBJMAP_MAX_VALUE 15 // 50000

#define UPDATE_FACTOR .06

class VelocityGrabber;

class SuperReceptiveField : public RateThread {

    int receptiveFieldsNum;

    vector< FOERecptiveField > receptiveFields;

    //Focus of Expansion
    int foeX;
	int foeY;



	//the patch of visual field with maximum value
	int maxPointX;
	int maxPointY;
	double maxPointProb;

    static VelocityBuffer velField;

    VelocityGrabber * velGrabber;

    string inPortName;
    string outPortName;

    yarp::sig::ImageOf <yarp::sig::PixelRgb> outImg;
    BufferedPort < yarp::sig::ImageOf <yarp::sig::PixelRgb> > outPort;

    yarp::sig::Matrix objMap;
    yarp::sig::Matrix objMapTS;
    unsigned long tSmoothTh; //15000;
    double tSmoothCoefficient;
    unsigned long tValidTh;  //100000;
    unsigned long sSmoothTh; //20000;
    int ttcMaxValue;

    void computeFoE();
    void makeObjMap();

    void visualizeObjMap();
    void visualizeFOE();


public:
    SuperReceptiveField(int receptFieldNum, unsigned long frameIntv, unsigned long smoothingIntv, int ttcRange);
    virtual ~SuperReceptiveField();


    bool threadInit();
    void threadRelease();
    void afterStart(bool success);

    void setPortNames(string inPName, string outPName);

    void static setVelField(VelocityBuffer velBuf){velField = velBuf;}


    void printFOEMAP();

    void run();
};




class VelocityGrabber : public BufferedPort<VelocityBuffer> {

    vector< FOERecptiveField > * receptiveFields;
    int receptiveFieldNum;
    SuperReceptiveField * superField;
    vector <Event * > dataMutexVec;

    bool start;


 public:
     VelocityGrabber(vector< FOERecptiveField > * rcptveFields/*, SuperReceptiveField * supField*/) {
         start = true ;
         receptiveFields = rcptveFields;
//       superField = supField;

         //set the mutex for all receptive fields
         receptiveFieldNum = receptiveFields -> size();
         dataMutexVec.reserve(receptiveFieldNum);
         Event * tmp;
         for (int i = 0; i < receptiveFields->size(); ++i) {
             tmp = new Event();
             dataMutexVec.push_back(tmp);
             (receptiveFields->at(i)).setDataMutex(tmp);
         }
     }

     void signalMutexs(){
         for (int i = 0; i < receptiveFieldNum; ++i) {
             (dataMutexVec.at(i))->signal();
         }
     }

    virtual void onRead(VelocityBuffer & data){

        if (start){
            //SuperReceptiveField::setVelField(data);
            //superField -> setVelField(data);
            FOERecptiveField::setVelField(data); //Set VelocityBuffer for all the threads

            signalMutexs(); // dataMutex -> signal();
            start = false;

        }else{
            if (FOERecptiveField::getDoneFlag() == receptiveFieldNum ){
                FOERecptiveField::resetDoneFlag();
                FOERecptiveField::setVelField(data); //Set VelocityBuffer for all the threads
                SuperReceptiveField::setVelField(data);
                signalMutexs(); //dataMutex -> signal();

            }
        }
    }



     virtual ~VelocityGrabber(){
       //delete dataMutex;
       for (int i = 0; i < receptiveFieldNum; ++i) {
          delete dataMutexVec.at(i);
       }
       dataMutexVec.clear();

       cout << "VelocityGrabber destructor done" << endl;
     }

};

#endif /* SUPERRECEPTIVEFIELD_H_ */
