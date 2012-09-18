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

#include <iCub/SuperReceptiveField.h>

VelocityBuffer SuperReceptiveField::velField;

SuperReceptiveField::SuperReceptiveField(int receptFieldNum):
       RateThread(10){
    receptiveFieldsNum = receptFieldNum;

    //reserve enough space for thread vectors
    receptiveFields.reserve(receptFieldNum);

    //initialize threads and mutexs
    int borderSize = FOEMAP_MAXREG_SIZE;
    for (int i = 0; i < receptFieldNum; ++i) {
        receptiveFields.push_back( FOERecptiveField(0, i * (RETINA_Y / receptFieldNum )
                                                     , RETINA_X - 1 + 2*borderSize
                                                     , (i + 1) * (RETINA_Y / receptFieldNum) -1 + 2*borderSize, borderSize ) );

    }


    objMap.resize(RETINA_X + 2*TTCNGHBR_SIZE, RETINA_Y + 2*TTCNGHBR_SIZE);
    //objMap.zero();
    for (int i = 0; i < RETINA_X; ++i) {
        for (int j = 0; j < RETINA_Y; ++j) {
            objMap(i,j) = OBJMAP_MAX_VALUE;
        }
    }
    objMapTS.resize(RETINA_X+ 2*TTCNGHBR_SIZE, RETINA_Y+ 2*TTCNGHBR_SIZE);
    objMapTS.zero();


    foeX = RETINA_X / 2;
    foeY = RETINA_Y / 2;

    velGrabber = new VelocityGrabber(&receptiveFields);

    outImg.resize(RETINA_X,RETINA_Y);
    outImg.zero();

}

bool SuperReceptiveField::threadInit(){
    //Open Input port
    if (!velGrabber->open( inPortName.c_str()) ) {
        cerr <<  "Heading Module: Sorry! Unable to open input port" << inPortName << endl;
        return false;
    }

    //Open output port
    if (!outPort.open(outPortName.c_str()) ){
       cerr <<  "Heading Module: Sorry! Unable to open input port" << outPortName << endl;
       return false;
    }

    //Start threads
    for (int i = 0; i < receptiveFieldsNum; ++i) {
        if (! receptiveFields.at(i).start() ){
            cerr <<  "Heading Module: Sorry! Unable to start threads" << endl;
            return false;
        }
    }

    velGrabber ->useCallback();

    return true;
}

void SuperReceptiveField::afterStart(bool success){
    if (!success){
        //close input port
        if (! velGrabber->isClosed())
            velGrabber -> close();

        //close output port
        if (!outPort.isClosed())
            outPort.close();

        //Stop the threads
        for (int j = 0; j < receptiveFieldsNum; ++j) {
            if (!receptiveFields.at(j).isRunning())
               receptiveFields.at(j).stop();
        }
    }
}

void SuperReceptiveField::threadRelease(){
    //Stop the threads

    for (int i = 0; i < receptiveFieldsNum; ++i) {
        receptiveFields.at(i).stop();
    }

    //Close the ports
     velGrabber -> close();
     outPort.close();

     cout << "Super thread released." << endl;
}

SuperReceptiveField::~SuperReceptiveField(){

	receptiveFields.clear();

	delete velGrabber;
	cout << "SuperReceptiveFiled destructor called." << endl;
}


void SuperReceptiveField::setPortNames(string inPName, string outPName){
    inPortName = inPName;
    outPortName = outPName;
}


void SuperReceptiveField::run(){

    int eventNo = velField.getSize();
    if (eventNo > 20){
//        computeFoE();

//        visualizeFOE();

        foeX = 0;
        foeY = 60;
        makeObjMap();
        visualizeObjMap();
    }

}


void SuperReceptiveField::computeFoE(){
	int recpFieldID = 0;

    //Step 1 & 2: Leaky Integration - Find the patch of visual field with maximum value

    //Collect the results of working threads and find the global maximum point
	maxPointProb = (receptiveFields.at(0)).getLocalFOEProb();
	for (int i = 1; i < receptiveFieldsNum; ++i) {
		if ( maxPointProb < (receptiveFields.at(i)).getLocalFOEProb() ){
		    maxPointProb = (receptiveFields.at(i)).getLocalFOEProb() ;
			recpFieldID = i;
		}
	}

	maxPointX = (receptiveFields.at(recpFieldID)).getLocalFOEX();
	maxPointY = (receptiveFields.at(recpFieldID)).getLocalFOEY();

    //Step 3: Shift the FOE toward the maximum patch

	//TODO: do not update if there are not enough number of events
    foeX =int ( foeX + UPDATE_FACTOR * (maxPointX - foeX) + .5 );
    foeY =int ( foeY + UPDATE_FACTOR * (maxPointY - foeY) + .5 );

    if (foeX < 0) foeX = 0;
    if (foeY < 0 ) foeY = 0;
    if (foeX > RETINA_X) foeX = RETINA_X;
    if (foeY > RETINA_Y) foeY = RETINA_Y;

}

void SuperReceptiveField::makeObjMap(){
    int size, x, y;
    double vx, vy, velNorm, distan, avg;
    double tim1, tim2, deltaT;
    double  normFactor, tmp, r, b , g;
    int wndwSZ;

    unsigned long tSmoothTh = 15000;
    double tSmoothCoefficient = .2;
    unsigned long tValidTh = 100000;
    unsigned long sSmoothTh = 20000;

    size = velField.getSize();

    unsigned long crntTS;

    int foeXCord = foeX + TTCNGHBR_SIZE;
    int foeYCord = foeY + TTCNGHBR_SIZE;
    //Update map with new arrived values
    for (int cntr = 0; cntr < size; ++cntr) {
       x = velField.getX(cntr) + TTCNGHBR_SIZE;
       y = velField.getY(cntr) + TTCNGHBR_SIZE;
       vx = velField.getVx(cntr);
       vy = velField.getVy(cntr);
       crntTS = velField.getTs(cntr);

       //velNorm = int ( sqrt(vx*vx + vy*vy) * 100000 + .5 ) + 1;
//       velNorm = int( sqrt(vx*vx + vy*vy )*10000  + .5 ); // velNorm = int( ( 1 / ( sqrt(vx*vx + vy*vy ) + .0000001 )) + .5 ); // 1 / (|V| + epsilon)
//       distan = sqrt( (foeXCord - x)*(foeXCord-x) + (foeYCord - y)*(foeYCord - y)); // distan = sqrt( (foeXCord - x)*(foeXCord-x) + (foeYCord - y)*(foeYCord - y));
//       distan = int (10000 / (distan + .0000001));


       velNorm = int( ( 1 / ( 100* sqrt(vx*vx + vy*vy ) + .0000001 )) + .5 ); // [ 1 / (|V| + epsilon) ]
       distan =  int (  sqrt( (foeXCord - x)*(foeXCord-x) + (foeYCord - y)*(foeYCord - y)) + .5) ;
       //if (distan < 15) distan = distan * 10 +10;


       deltaT = (crntTS - objMapTS(x, y) > tSmoothTh ? 0 : tSmoothCoefficient);
       objMap(x, y) =    (1 - deltaT) *  distan *  velNorm + deltaT * objMap(x, y) ;


       objMapTS(x, y) = crntTS;
   }

    //Smoothing
    crntTS = velField.getTs(size - 1);
    for (int x = TTCNGHBR_SIZE; x < RETINA_X + TTCNGHBR_SIZE; ++x) {
        for (int y = TTCNGHBR_SIZE; y < RETINA_Y+ TTCNGHBR_SIZE; ++y) {

           tim1 = objMapTS(x,y);

           if (crntTS - tim1 > tValidTh){
               objMap(x,y) = OBJMAP_MAX_VALUE;
               continue;
           }
           avg = 0;
           wndwSZ  =0;
           for (int i = -TTCNGHBR_SIZE; i <= TTCNGHBR_SIZE; ++i) {
               for (int j = -TTCNGHBR_SIZE; j <= TTCNGHBR_SIZE; ++j) {
                   tim2 = objMapTS(x + i, y + j);
                   deltaT = ( abs(tim1- tim2) > sSmoothTh ? 0 : 1 );
                   tmp = objMap(x + i, y + j) * deltaT;
                   avg += tmp;
                   wndwSZ  += deltaT;
              }
           }
           avg = avg / wndwSZ;
           objMap(x, y) = avg;
      }

//if (x > 50 && x < 80 && y > 50 && y < 80)
//    cout << avg << " ";

    }

//    cout << avg << endl;
//cout << endl;

   //Visualization

}

void SuperReceptiveField::visualizeFOE(){

    static const yarp::sig::PixelRgb r(200,0,0);
    static const yarp::sig::PixelRgb g(0,200,0);
    int eventNo = velField.getSize();

    int x, y;
    if (outPort.getOutputCount()) {
        yarp::sig::ImageOf <yarp::sig::PixelRgb>& imgSnd=outPort.prepare();
        imgSnd = outImg;
        for (int i = 0; i < eventNo; ++i) {
            x = velField.getX(i);
            y = velField.getY(i);
            imgSnd(x,y) = yarp::sig::PixelRgb(100, 100, 100);
        }
        yarp::sig::draw::addCircle(imgSnd,r,foeX,foeY,4);
        yarp::sig::draw::addCircle(imgSnd,g,maxPointX, maxPointY,4);

        outPort.write();
    }

}

void SuperReceptiveField::visualizeObjMap(){
    double  normFactor;
    int tempDepth, deltaT;
    double depthRed, depthGreen, depthBlue;

    int cntr;
    double TTC;

    yarp::sig::ImageOf<yarp ::sig::PixelRgb>& imgSnd=outPort.prepare();
    imgSnd.resize(RETINA_X ,RETINA_Y);
    imgSnd.zero();

    cntr =0; TTC = 0;

    normFactor = 255 / ((double) OBJMAP_MAX_VALUE);
    for (int i = 0; i < RETINA_X ; ++i) {
        for (int j = 0; j < RETINA_Y ; ++j) {

            //tempDepth = normFactor * objMap(i + TTCNGHBR_SIZE, j + TTCNGHBR_SIZE);

//            if (i > 35 && i < 55 && j > 30 && j < 85){
            if (i > 35 && i < 55 && j > 20 && j < 95){
               TTC +=  (normFactor * objMap(i + TTCNGHBR_SIZE, j + TTCNGHBR_SIZE) > 255 ? 255 : normFactor * objMap(i + TTCNGHBR_SIZE, j + TTCNGHBR_SIZE) );
//                TTC += objMap(i + TTCNGHBR_SIZE, j + TTCNGHBR_SIZE);
               cntr ++;
            }


            tempDepth = 255 - normFactor * objMap(i + TTCNGHBR_SIZE, j + TTCNGHBR_SIZE);
            tempDepth = (tempDepth < 0 ? 0 : tempDepth);

//            if (i > 35 && i < 55 && j > 30 && j < 85){
//               TTC += tempDepth;
//               cntr ++;
//            }


            if(tempDepth < 43){
                depthRed = tempDepth * 6;
                depthGreen = 0;
                depthBlue = tempDepth * 6;
            }
            if(tempDepth > 42 && tempDepth < 85){
                depthRed = 255 - (tempDepth - 43) * 6;
                depthGreen = 0;
                depthBlue = 255;
            }
            if(tempDepth > 84 && tempDepth < 128){
                depthRed = 0;
                depthGreen = (tempDepth - 85) * 6;
                depthBlue = 255;
            }
            if(tempDepth > 127 && tempDepth < 169){
                depthRed = 0;
                depthGreen = 255;
                depthBlue = 255 - (tempDepth - 128) * 6;
            }
            if(tempDepth > 168 && tempDepth < 212){
                depthRed = (tempDepth - 169) * 6;
                depthGreen = 255;
                depthBlue = 0;
            }
            if(tempDepth > 211 && tempDepth < 254){
                depthRed = 255;
                depthGreen = 255 - (tempDepth - 212) * 6;
                depthBlue = 0;
            }
            if(tempDepth > 253){
                depthRed = 255;
                depthGreen = 0;
                depthBlue = 0;
            }

            imgSnd(i,j) =  yarp::sig::PixelRgb ( depthRed, depthGreen , depthBlue );

            if (i > 35 && i < 55 && j == 20 && j == 95)
                imgSnd(i,j) =  yarp::sig::PixelRgb ( 255, 255 , 255 );

        }// end for on y dimention
    }// end for on x dimention
    TTC = TTC /cntr;

    cout << TTC << endl;

    static const yarp::sig::PixelRgb r(200,0,0);
    yarp::sig::draw::addCircle(imgSnd, r, foeX, foeY, 4);
    outPort.write();
}
