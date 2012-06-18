/*
 * FOEFinder.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: fuozhan
 */


#include  "FOEFinder.h"

FOEFinder::FOEFinder(){
	crnTS = 0;
	wrldStatus.resize(X_DIM, Y_DIM);
	wrldStatus.zero();
	tsStatus.resize(X_DIM, Y_DIM);
	tsStatus.zero();


	vxVels.resize(X_DIM, Y_DIM);
	vxVels.zero();
	vyVels.resize(X_DIM, Y_DIM);
    vyVels.zero();

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
                   wrldStatus(i,j) += .1;
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
                   wrldStatus(i,j) += .1;
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
               wrldStatus(i,j) += weight;
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
//			   wrldStatus(i,j) = LEAK_RATE * wrldStatus(i,j); //(crnTS - tsStatus(i,j)) * LEAK_RATE
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

    foeProb = 0;
    //Determine foe Position
    for (int i = 0; i < X_DIM; ++i) {
        for (int j = 0; j < Y_DIM; ++j) {
            if (wrldStatus(i, j) > foeProb){
                foeProb = wrldStatus(i, j);
                foeX = i;
                foeY = j;
            }
        }
    }

//    yarp::sig::ImageOf<yarp::sig::PixelFloat>& img=outPort-> prepare();
//    img.resize(X_DIM, Y_DIM);
//
//
//    float dis, vel, maxValue = 0, minValue=100000;
//    yarp::sig::Matrix ttcMatrix;
//    ttcMatrix.resize(X_DIM, Y_DIM);



    //visualise the FOE probabality over the image
//    img.zero();
//    normFactor = 254.0  / foeProb;
//    for (int i = 0; i < X_DIM; ++i) {
//        for (int j = 0; j < Y_DIM; ++j) {
//            img(i,j) =  normFactor  * wrldStatus(i,j) ;
//        }
//    }

//    static const yarp::sig::PixelFloat black=127;
//    //static const yarp::sig::PixelRgb red(2, 0, 2);
//    yarp::sig::draw::addCircle(img,black,foeX,foeY,4);
//
//    outPort -> write();
}

void FOEFinder::bin2(VelocityBuffer & data){
    int size, x,y, xs, xe, ys, ye, i, j;
    double aFlow, radian, vx, vy, a1, a2, b1, b2, dtmp1, dtmp2;
    double a3, b3;
    double normFactor, minValue, maxValue;


    radian = NGHBR_RADIAN; // angle between two consequative lines
    size = data.getSize();

    unsigned long tmpTS = data.getTs(size - 1);

//    cout << tmpTS << endl;

    //leak the values
//    for (int i = 0; i < X_DIM; ++i) {
//       for (int j = 0; j < Y_DIM; ++j) {
//           wrldStatus(i,j) = exp( - .00005* (tmpTS - tsStatus(i,j)) ) * wrldStatus(i,j); //LEAK_RATE * wrldStatus(i,j);
//       }
//    }


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
              wrldStatus(i,j) *= exp( - .00001* (data.getTs(cntr) - tsStatus(i,j)) );
              wrldStatus(i,j) +=  1000*sqrt(vx*vx + vy*vy); // 1 ;
              tsStatus(i,j) = data.getTs(cntr);
           } // end on y -coordinate
       } // end on x -coordinate
//        for (int i = 0; i < X_DIM; ++i) {
//            for (int j = 0; j < Y_DIM; ++j) {
//                tsStatus(i,j) = tmpTS;
//            }
//        }
//        	cout << x << " " << y << " " << vx << " " << vy << " " << data.getTs(cntr) << endl;

   }// end of loop on events


    if (size < 20)
       return;

   //Find the patch of visual fiel with maximum value
   double tmp, regfoeProb=0;
   int preFoeX, preFoeY;
   preFoeX = foeX;
   preFoeY = foeY;
   maxValue = 0;
   int windSz =  (2 * MAX_REG_NGHBR + 1)*(2 * MAX_REG_NGHBR + 1);
   for (int i = MAX_REG_NGHBR; i < X_DIM - MAX_REG_NGHBR; ++i) {
	  for (int j = MAX_REG_NGHBR; j < Y_DIM - MAX_REG_NGHBR; ++j) {

	      if (wrldStatus(i,j) > maxValue) // is needed for visualization
	          maxValue = wrldStatus(i,j);

		  tmp = 0;
		  for (int k = i - MAX_REG_NGHBR; k < i + MAX_REG_NGHBR + 1; ++k) { // i - MAX_REG_NGHBR + 2 * MAX_REG_NGHBR + 1 = i + MAX_REG_NGHBR + 1
			  for (int l = j - MAX_REG_NGHBR; l < j+MAX_REG_NGHBR + 1; ++l) {
				  tmp += wrldStatus(k,l);
			} // window -y
		  } // window - x
		  if (tmp > regfoeProb){
			  regfoeProb = tmp;
			  foeX = i;
			  foeY = j;
		  }
	  }
  }
  foeProb = regfoeProb;


  //shift teh FOE toward the maximum patch
  int tmp_x, tmp_y;
  tmp_x = foeX;
  tmp_y = foeY;

   foeX =int ( preFoeX + WEIGHT_FACTOR * (foeX - preFoeX) + .5 );
   foeY =int ( preFoeY + WEIGHT_FACTOR * (foeY - preFoeY) + .5 );

   if (foeX < 0) foeX = 0;
   if (foeY < 0 ) foeY = 0;
   if (foeX > X_DIM) foeX = X_DIM;
   if (foeY > Y_DIM) foeY = Y_DIM;


//   makeObjMap(data, foeX, foeY);

   yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=outPort-> prepare();
   img.resize(X_DIM, Y_DIM);
   img.zero();

   normFactor = 254 / maxValue;
   for (int i = 0; i < X_DIM; ++i) {
        for (int j = 0; j < Y_DIM; ++j) {
            img(i,j) = yarp::sig::PixelRgb (0,0 , normFactor * wrldStatus(i,j) );
        }
    }

   static const yarp::sig::PixelRgb pr(200,0,0);
   yarp::sig::draw::addCircle(img,pr,foeX,foeY,4);

   static const yarp::sig::PixelRgb pb(0,200,0);
   yarp::sig::draw::addCircle(img,pb,tmp_x,tmp_y,MAX_REG_NGHBR);

   outPort -> write();


}


void FOEFinder::makeObjMap(VelocityBuffer & data, int foeX, int foeY){
    int size, x, y;
    double vx, vy, velNorm, distan, leakyCons;
    static unsigned long lastTS = 0;
    unsigned long crntTS;
double maxValue, normFactor, tmp, r, b , g;


    size = data.getSize();

    maxValue = 0;
    crntTS = data.getTs(size -1);
    leakyCons = exp( -0.00001 * (crntTS - lastTS) );
    //leak the values
//    if (crnTS % 100 == 99){
        for (int i = 0; i < X_DIM; ++i) {
           for (int j = 0; j < Y_DIM; ++j) {
               objMap(i,j) = 0; //leakyCons * objMap(i,j); //LEAK_RATE * wrldStatus(i,j);

               if (objMap(i,j) > maxValue )
                   maxValue = objMap(i,j);

           }
        }
//        crnTS == 0;
//    }
//    crnTS ++;


    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);
        vx = data.getVx(cntr);
        vy = data.getVy(cntr);

        velNorm = sqrt(vx*vx + vy*vy) * 100;
        distan = sqrt( (foeX - x)*(foeX-x) + (foeY - y)*(foeY - y)) ;
        objMap(x, y) =  distan / velNorm; //

        if (maxValue < objMap(x,y))
            maxValue = objMap(x,y);

//        img(x,y) = yarp::sig::PixelRgb (1000*velNorm, distan , 100 * objMap(x,y) );
        //img(x,y) = yarp::sig::PixelRgb (100 * objMap(x,y), 0, 0 );

    }


    yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=outPort-> prepare();
    img.resize(X_DIM, Y_DIM);
    img.zero();

    //normFactor = (765/ maxValue) * 3;
    //normFactor = ((255*255*255)/ maxValue) *3 ;
    normFactor = (255/ maxValue);
    for (int i = 0; i < X_DIM; ++i) {
       for (int j = 0; j < Y_DIM; ++j) {
//            img(i,j) = yarp::sig::PixelRgb (10 * objMap(i,j),
//                                            100 * (objMap(i,j) > 1 ? objMap(i,j) - int (objMap(i,j)) : objMap(i,j) ),
//                                            1000 * (objMap(i,j) > .1 ? (objMap(i,j)*10 - int(objMap(i,j)*10 ))*.1 : objMap(i,j)) );


//           img(i,j) = yarp::sig::PixelRgb (normFactor * objMap(i,j), normFactor * objMap(i,j), normFactor * objMap(i,j));

           tmp = normFactor * objMap(i,j);
//           r = int (tmp / (255*255));
//           g = (tmp - r * (255*255)) / 255;
//           b = int (tmp)% 255;
//           if (r > 255){
//               r = g = b = 255;
//           }

           //img(i,j) = yarp::sig::PixelRgb (r,g , b);

           img(i,j) = yarp::sig::PixelRgb (tmp,tmp , tmp);

//           tmp = normFactor * objMap(i,j);
//           if (tmp < 255){
//               r = 0; g = 0; b = tmp;
//           }else{
//               if (tmp < 510) {
//                   r = 0; g = tmp - 255; b = 0;
//               } else {
//                   if (tmp < 765){
//                     r  = tmp - 510; g = 0; b = 0;
//                   }else{
//                       r = g = b = 255;
//                   }
//               }
//           }
//           img(i,j) = yarp::sig::PixelRgb (r, g, b);

        }
    }

    lastTS = data.getTs(size - 1);


    cout << "objmap : " << maxValue << endl;

    static const yarp::sig::PixelRgb pr(200,0,0);
    yarp::sig::draw::addCircle(img,pr,foeX,foeY,4);


    outPort -> write();
}


void FOEFinder::velNormal(VelocityBuffer & data){
    int size, x,y;
    int xs, xe, ys, ye;
    double vx, vy, ax, ay, avg;
    crnTS++;
    yarp::sig::Matrix dis;
    dis.resize(X_DIM, Y_DIM);
    dis.zero();

//    if (crnTS  == 5){
//        crnTS =0;
        wrldStatus.zero();
//    }


    ax = 100 / data.getVxMax();
    ay = 100 / data.getVyMax();

    double maxValue=0, minValue = 100000, normFactor;
    avg = 0;

    int foeX = 64, foeY = 64;

    size = data.getSize();
    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);
        vx = data.getVx(cntr) ;//* ax;
        vy = data.getVy(cntr) ;//* ay;
//        dis(x, y) = sqrt ( (foeX - x)*(foeX - x) + (foeY - y)*(foeY - y) );
        wrldStatus(x,y)= sqrt(vx*vx + vy*vy);

//        wrldStatus(x,y) = dis(x,y) / wrldStatus(x,y);
        cout << wrldStatus(x,y) << endl;
        avg += wrldStatus(x,y);
        if (wrldStatus(x,y) < minValue)
        	minValue = wrldStatus(x,y);
        if (wrldStatus(x,y) > maxValue)
        	maxValue = wrldStatus(x,y);
    }

    avg = avg / size;

 //   cout << minValue << " " << maxValue << " " << avg << " " << size << endl;



    maxValue = .15;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=outPort-> prepare();
    //copy wrldStatus Matrix to img
    img.resize(X_DIM, Y_DIM);
    double nf1, nf2, nf3, mv1, mv2, mv3;
    mv1 = .05;// 800; //
    mv2 = .08; // 900; //
    mv3 =  .25;// 1000; //
    nf1 = 254/mv1;
    nf2 = 254 /(mv2 - mv1);
    nf3 = 254 /(mv3 - mv2);
    normFactor = 254.0 / maxValue ;
    for (int i = 0; i < X_DIM; ++i) {
        for (int j = 0; j < Y_DIM; ++j) {

        	yarp::sig::PixelRgb p(0,0,0);


        	if (wrldStatus(i,j) <= mv1){
        		 p.g = int(nf1* wrldStatus(i,j) + .5); // (0, int(nf1* wrldStatus(i,j) + .5), 0);
//        		 if (p.g != 0)
//        		    p.g =  int ((dis(i,j)/ p.g) * 255);
        	}
        	else{
        		if (wrldStatus(i,j) > mv1 && wrldStatus(i,j) <= mv2){
					p.b =  int(nf2 * (wrldStatus(i,j) - mv1) + .5);
//					p.b =  int ((dis(i,j)/ p.b) * 255);
        		}
				else {
					if (wrldStatus(i,j) > mv2 && wrldStatus(i,j) <= mv3){
					   p.r = int(nf3 * (wrldStatus(i,j)-mv2) + .5);
//					   p.r =  int ((dis(i,j)/ p.r) * 255);
					}
				}
        	}

            img(i,j) = p;// int(normFactor * wrldStatus(i,j) + .5);
        }
    }
    outPort -> write();

}
void FOEFinder::velDivergance(VelocityBuffer & data){
    int size, x,y;
    int xs, xe, ys, ye;
    double vx, vy, ax, ay ;
    double vxx, vxy, vyy, vyx;
    double rVels = 0, lVels = 0;
    int rNum = 0, lNum = 0;
    //crnTS++;

//    if (crnTS  == 0){
//        crnTS =0;
//        vxVels.zero();
//        vyVels.zero();
//        wrldStatus.zero();
//    }

    double ttc = 0;

ax= 1;//   ax = 20 / data.getVxMax();
ay = 1; //   ay = 20 / data.getVyMax();

    size = data.getSize();
    for (int cntr = 0; cntr < size; ++cntr) {
       x = data.getX(cntr);
       y = data.getY(cntr);
       vxVels(x, y) = data.getVx(cntr) * ax;
       vyVels(x, y) = data.getVy(cntr) * ay;
    }


    for (int cntr = 0; cntr < size; ++cntr){
        x = data.getX(cntr);
        y = data.getY(cntr);

        vxx = data.getVx(cntr) * data.getVx(cntr);
        vyy = data.getVy(cntr)*data.getVy(cntr);
        if (x < 64){
            lVels += sqrt(vxx + vyy);
            lNum++;
        }else{
            rVels += sqrt(vxx + vyy);
            rNum++;
        }



//       vxx = sobelx(vxVels, x-1, y-1);
//        vyy = sobely(vyVels, x-1, y-1);
//        cout << vxx << " " << vyy << endl;
//        wrldStatus(x,y) = fabs(vxx + vyy);
//        ttc += fabs(vxVels(x,y)*vxVels(x,y) + vyVels(x,y)*vyVels(x,y));//wrldStatus(x,y);


    }

    //ttc = ttc / size;

    cout.precision(10);
//    cout << lVels << " " << lNum << " " << rVels << " " << rNum << endl;

//    cout << ttc << "  " << size << endl;
//    yarp::sig::ImageOf<yarp::sig::PixelFloat>& img=outPort-> prepare();
//    //copy wrldStatus Matrix to img
//    img.resize(X_DIM, Y_DIM);
//    img.zero();
//    for (int i = 0; i < X_DIM; ++i) {
//        for (int j = 0; j < Y_DIM; ++j) {
//
//           if (wrldStatus(i,j) != 0){
//              img(i,j) = int( ( wrldStatus(i,j)) + .5); //int( ( 2/wrldStatus(i,j)) * 100000);//
//              //cout << wrldStatus(i,j) << endl;
//           }
//
//        }
//    }
//
//    outPort -> write();


}


int FOEFinder::sobelx(yarp::sig::Matrix & mtx, int stR, int stC){
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
}


FOEFinder::~FOEFinder(){}

//Areas  2 | 1
//       -----
//       3 | 0
//        if (vx >= 0){
//            area = 0;
//            if (vy < 0)
//                area = 1;
//        }else {
//            area = 3;
//            if (vy < 0)}
//               area = 2;
//        }
//
//        switch (area) {
//            case 0:
//                xs = 0; xe = x;
//                for (i = xs; i < xe; ++i) {
//                    dtmp1 = a1 * i + b1;
//                    dtmp2 = a2 * i + b2;
//                    if (dtmp2 <= 0)
//                        continue;
//                    ys = dtmp1; ye = dtmp2;
//                    for (j = ys; j < ye; ++j) {
//                        if (j < 0)
//                            continue;
//                    }
//                }
//                break;
//            case 1:
//                xs = 0; xe = x;
//                for (i = xs; i < xe; ++i) {
//                    dtmp1 = a1 * i + b1;
//                    dtmp2 = a2 * i + b2;
//                    if (dtmp1 > 128)
//                        continue;
//                    ys = dtmp1; ye = dtmp2;
//                    for (j = ys; j < ye; ++j) {
//                        if (j >128)
//                            continue;
//                    }
//                }
//                break;
//            case 2:
//            xs = x; xe = X_DIM;
//            for (i = xs; i < xe; ++i) {
//                dtmp1 = a1 * i + b1;
//                dtmp2 = a2 * i + b2;
//                if (dtmp2 > 128)
//                    continue;
//                ys = dtmp2; ye = dtmp1;
//                for (j = ys; j < ye; ++j) {
//                    if (j >128)
//                       continue;
//                }
//            }
//            break;
//            case 3:
//            xs = x; xe = X_DIM;
//            for (i = xs; i < xe; ++i) {
//                dtmp1 = a1 * i + b1;
//                dtmp2 = a2 * i + b2;
//                if (dtmp1 < 0 )
//                    continue;
//                ys = dtmp2; ye = dtmp1;
//                for (j = ys; j < ye; ++j) {
//                    if (j < 0)
//                       continue;
//                }
//            }
//            break;
//
//
//
//            default:
//                break;
//        }


//negative weight
//       xs = x; xe = X_DIM;
//     if (vx < 0){
//         xs = 0; xe = x;
//     }
//
//       a3 = tan(aFlow + (M_PI / 2));
//       b3 = y - a3 * x;
//
//       if (vy > 0 ){
//         for (i = xs; i < xe; ++i) {
//             ys = a3*i + b3;
//             ys = (ys < 0 ? 0 : ys);
//             ye = Y_DIM;
//             for (j = ys; j < ye; ++j) {
//                 wrldStatus(i,j) -= .1;
//                 wrldStatus(i,j)= (wrldStatus(i,j) < 0 ? 0 : wrldStatus(i, j));
//             }
//         }
//       } // end if vy > 0
//       else {
//         for (i = xs; i < xe; ++i) {
//             ys = 0;
//             ye = a3*i + b3;
//             ye = (ye > Y_DIM ? Y_DIM : ye);
//             for (j = ys; j < ye; ++j) {
//                 wrldStatus(i,j) -= .1;
//                 wrldStatus(i,j)= (wrldStatus(i,j) < 0 ? 0 : wrldStatus(i, j));
//             }
//         }
//       }// end else -- vy < 0


