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

#ifndef FOEReceptiveField_H_
#define FOEReceptiveField_H_

#include <cmath>
#include <iostream>

#include <yarp/os/Event.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Semaphore.h>
//#include <iCub/emorph/VelocityBuffer.h>
#include <iCub/VelocityBuffer.h>

using namespace std;
using namespace yarp::os;

#define NGHBR_RADIAN (M_PI / 10)  //used by "bin2" function  //The neihboring arc
#define LEAK_RATE (- 0.00001)

class FOERecptiveField : public Thread{
	int id; 
	int leftUpX;   //the x-coordinate of left upper pixel in the receptive field
	int leftUpY;   //the y-coordinate of left upper pixel in the receptive field
	int rightBotX; //the x-coordinate of right bottom pixel in the receptive field
	int rightBotY; //the y-coordinate of right bottom pixel in the receptive field

	int fieldXLength; // x-length of receptive field
	int fieldYLength; // y-length of receptive field

	yarp::sig::Matrix  foeMap;
	yarp::sig::Matrix  tsStatus;
	
	int borderSize;

	int localFOEX;
	int localFOEY;
	int foeProbablity;

	Event * dataAvaMutex;


	void updateWeights();
	void findLocalMax();

	static VelocityBuffer velField;
	static int doneFlag;
	static Semaphore doneFlagSemaphore;
	static int counter;

	inline void incrementDoneFlag();


public:
	FOERecptiveField(int lux, int luy, int rbx, int rby, int borderSize);
	FOERecptiveField(const FOERecptiveField & src);
	virtual ~FOERecptiveField();

	virtual bool threadInit();
//	void threadRelease();
	virtual void onStop();

	virtual void run();


	void setDataMutex(Event * dataMutex){ dataAvaMutex = dataMutex;}
	void static setVelField(VelocityBuffer velBuf){velField = velBuf;}


	static void resetDoneFlag();
    static int getDoneFlag(){return doneFlag;}

	int getLocalFOEX(){return localFOEX - borderSize;}
	int getLocalFOEY(){return localFOEY - borderSize;}
	int getLocalFOEProb(){return foeProbablity;}
	void printFOEMap();

};


#endif /* FOEReceptiveField_H_ */
