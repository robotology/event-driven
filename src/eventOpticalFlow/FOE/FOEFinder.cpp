/*
 * FOEFinder.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: fuozhan
 */


#include  "FOEFinder.h"

FOEFinder::FOEFinder(){
	crnTS = 0;
	foeMap.resize(X_DIM, Y_DIM);
	foeMap.zero();
	tsStatus.resize(X_DIM, Y_DIM);
	tsStatus.zero();

    objMap.resize(X_DIM, Y_DIM);
    objMap.zero();

    foeProb = 0;
    foeX = 64; foeY = 64;
    crnTS = 0;
    initBins(BIN_NO);
}

void FOEFinder::setOutPort(BufferedPort<yarp::sig::ImageOf
		                           <yarp::sig::PixelRgb> > * oPort){
	outPort = oPort;
}

void FOEFinder::computeFoE(VelocityBuffer & vb, bool vis){


    //Step 1: Leaky Integaration
    bin2(vb);

    //Step 2: Find the patch of visual fiel with maximum value
   int centerX, centerY;
   double foeMaxValue;
   getFOEMapMax(centerX, centerY,  foeMaxValue);

   //Step 3: Shift teh FOE toward the maximum patch
   if (vb.getSize() < 20){
       foeX =int ( foeX + WEIGHT_FACTOR * (centerX - foeX) + .5 );
       foeY =int ( foeY + WEIGHT_FACTOR * (centerY - foeY) + .5 );

       if (foeX < 0) foeX = 0;
       if (foeY < 0 ) foeY = 0;
       if (foeX > X_DIM) foeX = X_DIM;
       if (foeY > Y_DIM) foeY = Y_DIM;
   }


   //Send the FOE map to output port
   if (vis){
       yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=outPort-> prepare();
       img.resize(X_DIM, Y_DIM);
       img.zero();


    //   double normFactor = 254 / foeMaxValue;
    //   for (int i = 0; i < X_DIM; ++i) {
    //        for (int j = 0; j < Y_DIM; ++j) {
    //            img(i,j) = yarp::sig::PixelRgb (0,0 , normFactor * foeMap(i,j) );
    //        }
    //    }

       for (int i = 0; i < vb.getSize(); ++i) {
           img(vb.getX(i), vb.getY(i)) = yarp::sig::PixelRgb ( 100, 100, 100 );
       }

       static const yarp::sig::PixelRgb pr(200,0,0);
       yarp::sig::draw::addCircle(img,pr,foeX,foeY,4);

       static const yarp::sig::PixelRgb pb(0,200,0);
       yarp::sig::draw::addCircle(img,pb,centerX,centerY,FOEMAP_MAXREG_SIZE);

       outPort -> write();
   }
}


void FOEFinder::bin2(VelocityBuffer & data){
    int size, x,y, xs, xe, ys, ye, i, j;
    double aFlow, radian, vx, vy, a1, a2, b1, b2, dtmp1, dtmp2;
    double a3, b3;

    radian = NGHBR_RADIAN; // angle between two consequative lines
    size = data.getSize();

    unsigned long tmpTS = data.getTs(0);
//    cout << tmpTS << endl;

    //leak the values
    for (int i = 0; i < X_DIM; ++i) {
       for (int j = 0; j < Y_DIM; ++j) {
           foeMap(i,j) = exp( LEAK_RATE * (tmpTS - tsStatus(i,j)) ) * foeMap(i,j); //LEAK_RATE * foeMap(i,j);
           tsStatus(i,j) = tmpTS;
       }
    }

    //Leaky Integaration
    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);
        vx = data.getVx(cntr);
        vy = data.getVy(cntr);

        if (vx == 0 && vy ==0)
             continue;

        aFlow = atan2(vy, vx);
        //Calculate the slope of the flow
        a2 = tan(aFlow + radian);
        a1 = tan(aFlow - radian);
        b2 = y - a2 * x;
        b1 = y - a1 * x;

        //positive weight
        xs = 0; xe = x;
        if (vx < 0){
           xs = x; xe = X_DIM;
        }
        for (i = xs; i < xe; ++i) {
           dtmp1 = a1 * i + b1;
           dtmp2 = a2 * i + b2;
           ys =  dtmp1 + dtmp2 - fabs(dtmp1 - dtmp2); ys = ys /2;// ys is set to min(dtmp1, dtmp2)
           ye =  dtmp1 + dtmp2 + fabs(dtmp1 - dtmp2); ye = ye /2;// ye is set to max(dtmp1, dtmp2)
           if (ys >= 128 || ye < 0)
               continue;
           for (j = ys; j < ye; ++j) {
              if (j < 0 || j >= 128)
                  continue;
//              foeMap(i,j) *= exp( LEAK_RATE* (data.getTs(cntr) - tsStatus(i,j)) );
              foeMap(i,j) +=   .1; //1000*sqrt(vx*vx + vy*vy); // 1 ;
              tsStatus(i,j) = data.getTs(cntr);
           } // end on y -coordinate
       } // end on x -coordinate

//          cout << x << " " << y << " " << vx << " " << vy << " " << data.getTs(cntr) << endl;

   }// end of loop on events
}


void FOEFinder::getFOEMapMax(int & centerX, int & centerY, double & foeMaxValue){
    double tmp, regfoeProb=0;
    foeMaxValue = 0;
//    int windSz =  (2 * FOEMAP_MAXREG_SIZE + 1)*(2 * FOEMAP_MAX_REG_SIZE + 1);
    for (int i = FOEMAP_MAXREG_SIZE; i < X_DIM - FOEMAP_MAXREG_SIZE; ++i) {
       for (int j = FOEMAP_MAXREG_SIZE; j < Y_DIM - FOEMAP_MAXREG_SIZE; ++j) {

           if (foeMap(i,j) > foeMaxValue) // is needed for visualization
               foeMaxValue = foeMap(i,j);

           tmp = 0;
           for (int k = i - FOEMAP_MAXREG_SIZE; k < i + FOEMAP_MAXREG_SIZE + 1; ++k) { // i - MAX_REG_NGHBR + 2 * MAX_REG_NGHBR + 1 = i + MAX_REG_NGHBR + 1
               for (int l = j - FOEMAP_MAXREG_SIZE; l < j+FOEMAP_MAXREG_SIZE + 1; ++l) {
                   tmp += foeMap(k,l);
             } // window -y
           } // window - x
           if (tmp > regfoeProb){
               regfoeProb = tmp;
               centerX = i;
               centerY = j;
           }
       }
   }
   foeProb = regfoeProb;
}



void FOEFinder::initBins(int binNo){
    float radian;

    radian = M_PI / (2 * binNo); // angle between two consequative lines

    binSlopes.reserve(2*binNo + 1); // reserve enough space for line slopes

    binSlopes.push_back( log(0) ); // push -infinity as tan (-Pi/2)

    //calculate the line slpoes and save them in binSlopes
    for (int i = -(binNo - 1); i < binNo; ++i) {
        binSlopes.push_back( tan(i * radian) );
    }

    binSlopes.push_back( -log(0) ); // push infinity as tan (Pi/2)

}

void FOEFinder::populateBins(int idx1, int idx2, int x, int y, double vx, double vy, double weight){
    int xs, xe, ys, ye, i,j;
    double a1,a2, b1, b2, dtmp1, dtmp2;

    //find the bordering line parameters (slopes: a1, a2 ,and yIntercept: b1, b2)
    a1 = binSlopes[idx1];
    a2 = binSlopes[idx2];
    b1 = y - a1 * x;
    b2 = y - a2 * x;

    //Increase the potential for points below the velocity vector and follow the maximum value
    xs = 0; xe = x;
    if (vx < 0){
       xs = x; xe = X_DIM;
    }

    if (idx1 == 0 || idx2 == 2 * BIN_NO){  //One of the borders is a vertical line (slop of infinity)

       if (vy >= 0){
           ys = 0;
           a1 = (idx1 == 0 ? a2 : a1);
           b1 = (idx1 == 0 ? b2 : b1);
           for (i = xs; i < xe; ++i) {
               ye = a1 * i + b1;
               if (ye < 0)
                   break;
               for (j = ys; j < ye; ++j) {
                   foeMap(i,j) += .1;
               }
           }
       }else {  // vy < 0
           ye = Y_DIM;
           a1 = (idx1 == 0 ? a2 : a1);
           b1 = (idx1 == 0 ? b2 : b1);
           for (i = xs; i < xe; ++i) {
               ys = a1 * i + b1;
               if (ys > 128)
                   break;
               for (j = ys; j < ye; ++j) {
                   foeMap(i,j) += .1;
               }
           }
       }

   } else { //Non of the bordering lines has a slope of infinity
       for (i = xs; i < xe; ++i) {
           dtmp1 = a1 * i + b1;
           dtmp2 = a2 * i + b2;
           ys =  dtmp1 + dtmp2 - fabs(dtmp1 - dtmp2); ys = ys /2;// ys is set to min(dtmp1, dtmp2)
           ye =  dtmp1 + dtmp2 + fabs(dtmp1 - dtmp2); ye = ye /2;// ye is set to max(dtmp1, dtmp2)
           if (ys >= 128 || ye < 0)
               continue;
           for (j = ys; j < ye; ++j) {
               if (j < 0 || j >= 128)
                   continue;
               foeMap(i,j) += weight;
            } // end on y -coordinate
       } // end on x -coordinate
   }// end for else

}

void FOEFinder::bin(VelocityBuffer & data){
    int size, x,y;
    double vx, vy, aEvent;
    float normFactor;
    int foeNo = 1;
    int idx1, idx2;

    //leak the values
//    if (crnTS % 100 == 99){
//		for (int i = 0; i < X_DIM; ++i) {
//		   for (int j = 0; j < Y_DIM; ++j) {
//			   foeMap(i,j) = LEAK_RATE * foeMap(i,j); //(crnTS - tsStatus(i,j)) * LEAK_RATE
//		   }
//		}
//		crnTS = 0;
//	}
//
//	crnTS++;



    //iterate over velocity events
    size = data.getSize();
    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);
        vx = data.getVx(cntr);
        vy = data.getVy(cntr);

        if (vx == 0 && vy ==0)
             continue;

         //Calculate the slope of the flow
        if (vx != 0)
           aEvent = vy / vx;
        else
            aEvent = -vy * log(0); // TODO

        //search the bin and two boundary lines
        idx1 = 0;
        idx2= 2*BIN_NO;
        for (int i = 1; i < 2*BIN_NO; ++i) {
            if (binSlopes[i] > aEvent){
                idx2 = i;
                break;
            }
            idx1 = i;
        }

        populateBins(idx1, idx2, x, y, vx, vy, 1);

    }// end-for iteration on events

}



void FOEFinder::makeObjMap(VelocityBuffer & data){
    int size, x, y;
    double vx, vy, velNorm, distan, leakyCons;
    static unsigned long lastTS = 0;
    unsigned long crntTS;
    double maxValue, sptDrv, normFactor, tmp, r, b , g;

    size = data.getSize();

    maxValue = 0;
    crntTS = data.getTs(0);
    leakyCons = exp( -0.00001 * (crntTS - lastTS) );
    //leak the values
    for (int i = 0; i < X_DIM; ++i) {
       for (int j = 0; j < Y_DIM; ++j) {
           objMap(i,j) = leakyCons * objMap(i,j); //LEAK_RATE * foeMap(i,j);

//           if (objMap(i,j) > maxValue )
//               maxValue = objMap(i,j);
       }
    }


maxValue  = 0; sptDrv = 0;
    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);
        vx = data.getVx(cntr);
        vy = data.getVy(cntr);

        velNorm = sqrt(vx*vx + vy*vy) * 10000;
        distan = sqrt( (foeX - x)*(foeX-x) + (foeY - y)*(foeY - y)) ;
        objMap(x, y) =  distan / velNorm;


        maxValue += objMap(x, y);
        sptDrv += objMap(x, y) * objMap(x, y);
    }
    maxValue = maxValue / size;
    sptDrv = sptDrv / size;
    sptDrv = sptDrv - maxValue * maxValue;
    sptDrv = sqrt(sptDrv);
    cout << maxValue << endl;

    maxValue +=  sptDrv;



    yarp::sig::ImageOf<yarp ::sig::PixelRgb>& img=outPort-> prepare();
    img.resize(X_DIM, Y_DIM);
    img.zero();

    normFactor = (255/ (.3 * maxValue));
    for (int i = 0; i < X_DIM; ++i) {
       for (int j = 0; j < Y_DIM; ++j) {

           if (objMap(i, j) > .7 * maxValue){
               r = normFactor * (objMap(i, j) - .7 * maxValue);
               img(i,j) =  yarp::sig::PixelRgb ( r, 0 , 0 );
           }else {
               if (objMap(i, j) > .3 * maxValue ){
                   b = normFactor * (objMap(i, j) - .3 * maxValue);
                   img(i,j) =  yarp::sig::PixelRgb ( 0, 0 , b );
               }
               else{
                   g = normFactor * objMap(i, j) ;
                   img(i,j) =  yarp::sig::PixelRgb ( 0, g , 0 );
               }
           }
//        tmp = normFactor * objMap(i,j);
//        img(i,j) = yarp::sig::PixelRgb (tmp,tmp , tmp);
        }
    }

    lastTS = data.getTs(0);



    static const yarp::sig::PixelRgb pr(200,0,0);
    yarp::sig::draw::addCircle(img,pr,foeX,foeY,4);


    outPort -> write();
}


FOEFinder::~FOEFinder(){}

/*int FOEFinder::sobelx(yarp::sig::Matrix & mtx, int stR, int stC){
    int res = 0;
    res = (mtx(stR, stC + 2) +  mtx(stR+1, stC+2) + mtx(stR+2, stC+2))
            - (mtx(stR, stC) +  mtx(stR+1, stC) + mtx(stR+2, stC));
    return res;

}

int FOEFinder::sobely(yarp::sig::Matrix & mtx, int stR, int stC){
    int res = 0;
    res = (mtx(stR+2, stC ) +  mtx(stR+2, stC+1) + mtx(stR+2, stC+2))
            - (mtx(stR, stC ) +  mtx(stR, stC+1) + mtx(stR, stC+2));

    return res;
}*/

