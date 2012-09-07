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


#include <iCub/FOERecptiveField.h>


VelocityBuffer FOERecptiveField::velField;
Semaphore FOERecptiveField::doneFlagSemaphore (1);
int FOERecptiveField::doneFlag = 0;
int FOERecptiveField::counter = 0;

FOERecptiveField::FOERecptiveField(int lux, int luy, int rbx, int rby, int borderSize):
       Thread(){

    id = counter++;

    leftUpX = lux;
    leftUpY = luy;
    rightBotX = rbx;
    rightBotY = rby;
    this->borderSize = borderSize;

    fieldXLength = rightBotX - leftUpX + 1;
    fieldYLength = rightBotY - leftUpY + 1;

    localFOEX = 0; localFOEY = 0;
}


FOERecptiveField::FOERecptiveField(const FOERecptiveField & src){
    id = src.id;
    leftUpX = src.leftUpX;
    leftUpY = src.leftUpY;
    rightBotX = src.rightBotX;
    rightBotY = src.rightBotY;

    borderSize = src.borderSize;
    fieldXLength = src.fieldXLength;
    fieldYLength = src.fieldYLength;

    //matrix foeMap, tsStatus
    foeMap = yarp::sig::Matrix(src.foeMap);
    tsStatus = yarp::sig::Matrix(src.tsStatus);

    localFOEX = src.localFOEX;
    localFOEY = src.localFOEY;
    foeProbablity = src.foeProbablity;

    dataAvaMutex = src.dataAvaMutex;
}

FOERecptiveField::~FOERecptiveField(){
    cout << "FOEReceptive Filed destructor called." << endl;
}

bool FOERecptiveField::threadInit(){
    foeMap.resize(fieldXLength, fieldYLength);
    tsStatus.resize(fieldXLength, fieldYLength);
    foeMap.zero();
    tsStatus.zero();
    return true;
}

//void FOERecptiveField::threadRelease(){
//foeMap.resize(0,0);
//tsStatus.resize(0,0);
//
//}

void FOERecptiveField::onStop(){
    dataAvaMutex -> signal();
}

void FOERecptiveField::run(){
	int evnNo;

	while (!isStopping()){ // while the thread is not stopped continue working

	    dataAvaMutex -> wait(); // wait until data arrived on the port

		if (isStopping())
			break;

		//Step 1: Leaky Integration
		updateWeights();
		//Step 2: Find the patch of visual field with maximum value
		findLocalMax();

		incrementDoneFlag();
	}

	cout << "finished " << id << endl;

}

void FOERecptiveField::resetDoneFlag(){
    doneFlagSemaphore.wait();
    doneFlag = 0;
    doneFlagSemaphore.post();
}

void FOERecptiveField::incrementDoneFlag(){
    doneFlagSemaphore.wait();
    doneFlag++;
    doneFlagSemaphore.post();
}


void FOERecptiveField::updateWeights(){
    int evnNo, x,y, xs, xe, ys, ye, i, j;
    double aFlow, radian, vx, vy, a1, a2, b1, b2, dtmp1, dtmp2;
    double a3, b3;

    radian = NGHBR_RADIAN; // angle between two consequative lines
    evnNo = velField.getSize();



    //leak the values
    if (evnNo > 0) {
        unsigned long tmpTS = velField.getTs(0); unsigned long tmpUL;
        for (int i = 0; i < fieldXLength; ++i) {
           for (int j = 0; j < fieldYLength; ++j) {
               tmpUL = tsStatus(i,j);
               foeMap(i,j) = exp( LEAK_RATE * (tmpTS - tmpUL ) ) * foeMap(i,j); //LEAK_RATE * foeMap(i,j);
               tsStatus(i,j) = tmpTS;
           }
        }
    }

    //Leaky Integaration
    for (int cntr = 0; cntr < evnNo; ++cntr) {
        x = velField.getX(cntr) + borderSize;
        y = velField.getY(cntr) + borderSize;
        vx = velField.getVx(cntr);
        vy = velField.getVy(cntr);

        if (vx == 0 && vy ==0)
             continue;

        aFlow = atan2(vy, vx);
        //Calculate the slope of the flow
        a2 = tan(aFlow + radian);
        a1 = tan(aFlow - radian);
        b2 = y - a2 * x;
        b1 = y - a1 * x;

        //positive weight
        xs = leftUpX; xe = x;
        if (vx < 0){
           xs = x; xe = rightBotX;
        }

        if (xs < leftUpY ) xs = leftUpY; if (xe > rightBotY) xe = rightBotY; //check the field boundary - x

        for (i = xs; i <= xe; ++i) {
           dtmp1 = a1 * i + b1;
           dtmp2 = a2 * i + b2;
           ys =  dtmp1 + dtmp2 - fabs(dtmp1 - dtmp2); ys = ys /2;// ys is set to min(dtmp1, dtmp2)
           ye =  dtmp1 + dtmp2 + fabs(dtmp1 - dtmp2); ye = ye /2;// ye is set to max(dtmp1, dtmp2)
           if (ys > rightBotY || ye < leftUpY)
               continue;

           if (ys < leftUpY ) ys = leftUpY; if (ye > rightBotY) ye = rightBotY; // check the field boundary- y

           for (j = ys; j <= ye; ++j) {
//              foeMap(i,j) *= exp( LEAK_RATE* (data.getTs(cntr) - tsStatus(i,j)) );
              foeMap( i - leftUpX, j - leftUpY) +=   .1; //1000*sqrt(vx*vx + vy*vy); // 1 ;
              tsStatus(i - leftUpX, j - leftUpY) = velField.getTs(cntr);
           } // end on y -coordinate
       } // end on x -coordinate
   }// end of loop on events

}

void FOERecptiveField::findLocalMax(){
    double tmp, regfoeProb=0;

//    int windSz =  (2 * FOEMAP_MAXREG_SIZE + 1)*(2 * FOEMAP_MAX_REG_SIZE + 1);
    for (int i = leftUpX + borderSize; i < rightBotX - borderSize + 1; ++i) {
       for (int j = leftUpY + borderSize; j < rightBotY - borderSize + 1; ++j) {

           tmp = 0;
           for (int k = i - borderSize; k < i + borderSize+ 1; ++k) { // i - MAX_REG_NGHBR + 2 * MAX_REG_NGHBR + 1 = i + MAX_REG_NGHBR + 1
               for (int l = j - borderSize; l < j+borderSize+ 1; ++l) {
                   tmp += foeMap(k - leftUpX, l - leftUpY);
             } // window -y
           } // window - x
           if (tmp > regfoeProb){
               regfoeProb = tmp;
               localFOEX = i;
               localFOEY = j;
           }
       }
   }
   foeProbablity = regfoeProb;
}


