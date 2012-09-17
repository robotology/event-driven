#include "flowViewer.h"
#include <iostream>

flowViewer::flowViewer(int visMethod )
    : RateThread(30) {

    vecBaseImg.resize(4*XDIM,4*YDIM);

    IMGFOR(vecBaseImg ,i , j) {
        vecBaseImg(i, j) = 150;
    }

    normBaseImg.resize(XDIM, YDIM);
    IMGFOR(normBaseImg ,i , j) {
        normBaseImg(i, j) = 150;
    }

    xVels.resize(XDIM + 2*FILTER_NGHBRHD, YDIM + 2*FILTER_NGHBRHD);    // xVels.resize(XDIM, YDIM);
    xVels.zero();
    yVels.resize(XDIM + 2*FILTER_NGHBRHD, YDIM + 2*FILTER_NGHBRHD);    // yVels.resize(XDIM, YDIM);
    yVels.zero();
    timeRec.resize(XDIM + 2*FILTER_NGHBRHD, YDIM + 2*FILTER_NGHBRHD);   //resize(XDIM, YDIM);
    timeRec.zero();

    visMthd = visMethod;

//    zeroFlag = 0;
//    cntrR = 0; cntrL = 0;
//    velR = 0; velL = 0;


}

void flowViewer::
setPorts(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * oPort, VelocityGrabber * iPort, BufferedPort<Bottle> * oDataPort){
    inPort = iPort;
    outImagePort = oPort;
	outDataPort = oDataPort;
}

flowViewer::~flowViewer()
{
}

void flowViewer::run(){

    VelocityBuffer * vb;
    static double last_read = 0;
    vb = inPort->getVelocities();

    if (vb != NULL){
       if (vb->getSize() > 10){
           avrageFilter(*vb);
           switch (visMthd) {
              case 1:
                  velVect(*vb);
                  break;
              case 2:
                  velNorm(*vb);
                  break;
           }
      }
      delete vb;
      last_read = yarp::os::Time::now();
    } 
    else
    {
       double curr_time = yarp::os::Time::now();
       if (curr_time-last_read> 0.030*3 )
       {
           yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outImagePort-> prepare();
           img=vecBaseImg;
           outImagePort->write();
           last_read = curr_time;
       }
    }
}

void flowViewer::avrageFilter(VelocityBuffer& packet){
    int x, y;
    double vx, vy;
    double xVelAvg, yVelAvg, tim1, tim2 , deltaT;
    double xVelSptDer, yVelSptDer, tmp;
    //int wndwSZ = (2*FILTER_NGHBRHD + 1); //*(2*FILTER_NGHBRHD + 1);
    int wndwSZ;


    int packetSize =  packet.getSize();
    // Update Values with the ariving packet
    for (int idx = 0; idx < packetSize; ++idx) {
        x = packet.getX(idx) + FILTER_NGHBRHD;
        y = packet.getY(idx) + FILTER_NGHBRHD;

        timeRec(x,y) = packet.getTs(idx);
        xVels(x,y) = packet.getVx(idx);
        yVels(x,y) = packet.getVy(idx);
    }

    //Apply the average Filter
    for (int idx = 0; idx < packetSize; ++idx) {
        x = packet.getX(idx) + FILTER_NGHBRHD;
        y = packet.getY(idx) + FILTER_NGHBRHD;

       xVelAvg = 0; yVelAvg = 0;
       xVelSptDer = 0; yVelSptDer = 0;
       tim1 = timeRec(x,y);
       wndwSZ  =0;
       for (int i = -FILTER_NGHBRHD; i <= FILTER_NGHBRHD; ++i) {
           for (int j = -FILTER_NGHBRHD; j <= FILTER_NGHBRHD; ++j) {
               tim2 = timeRec(x + i, y + j);
//               deltaT = (abs(tim1- tim2)!= 0 ? abs(tim1- tim2) : 1);
//               xVel += (xVels(x + i, y + j) / deltaT);
//               yVel += (yVels(x + i, y + j) / deltaT);
               deltaT = ( abs(tim1- tim2) > 30000 ? 0 : 1 );
               tmp = xVels(x + i, y + j) * deltaT;
               xVelAvg += tmp;
               xVelSptDer += tmp* tmp;
               tmp = yVels(x + i, y + j) * deltaT;
               yVelAvg += tmp;
               yVelSptDer += tmp * tmp;
               wndwSZ  += deltaT;
          }
       }
       xVelAvg = xVelAvg / wndwSZ;
       yVelAvg = yVelAvg / wndwSZ;
       xVelSptDer = xVelSptDer / wndwSZ;
       yVelSptDer = yVelSptDer / wndwSZ;
       xVelSptDer = sqrt(xVelSptDer - xVelAvg * xVelAvg);
       yVelSptDer = sqrt(yVelSptDer - yVelAvg * yVelAvg);
       vx = packet.getVx(idx);
       vy = packet.getVy(idx);

       if ( xVelAvg + 2*xVelSptDer < vx || vx < xVelAvg - 2*xVelSptDer )
           xVelAvg = 0;
       if ( yVelAvg + 2*yVelSptDer < vy || vy < yVelAvg - 2*yVelSptDer )
           yVelAvg = 0;

       packet.setVx(idx, xVelAvg);
       packet.setVy(idx, yVelAvg);

    }

}

void flowViewer::velVect(VelocityBuffer& data)
{
    double average, sptDer;
    int x, y;
    double vx, vy;
    int hx,hy, X,Y;

    double ax, bx, ay, by;
    double vvx, vvy;

    int size;
    double norm, tmp;
    yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outImagePort-> prepare();
    img=vecBaseImg;

    size = data.getSize();

    ax =  (data.getVxMax() == 0 ? 1 : 40 / data.getVxMax());
    ay =  (data.getVyMax() == 0 ? 1 : 40 / data.getVyMax());

    average = 0; sptDer = 0;
    for (int i = 0; i < size; ++i) {
        x = data.getX(i);
        y = data.getY(i);
        vx = data.getVx(i);
        vy = data.getVy(i);

		data_x[x][y] = vx;
		data_y[x][y] = vy;
        vvx = ax * vx;
        vvy = ay * vy;

        norm=vvx*vvx+vvy*vvy;

        if (norm >= 0.0){
            X=2+4*x;
            Y=2+4*y;

            hx=X+ (vvx >= 0 ? int(vvx+0.5) : int(vvx - 0.5) ) ;
            hy=Y+ (vvy >= 0 ? int(vvy+0.5) : int(vvy - 0.5) ) ;

            static const yarp::sig::PixelMono16 black=0;
            static const yarp::sig::PixelMono16 white=255;

            yarp::sig::draw::addSegment(img,black,X,Y,hx,hy);
            yarp::sig::draw::addCircle(img,black,hx,hy,2);


        }

//        average += norm;
//        sptDer += norm * norm;
    }

//    average = average / size;
//    sptDer = sptDer / size;
//    sptDer = sptDer - average * average;
 //   cout << average << " " << sptDer << endl;


    outImagePort->write();
//    cout << "after write " << endl;

	//send the data
	if (outDataPort->getOutputCount()>0)
	{
		Bottle &b = outDataPort->prepare();
		b.addInt(XDIM);
		b.addInt(YDIM);
		for (int y=0; y<YDIM; y++)	
			for (int x=0; x<XDIM; x++)
				{
					b.addDouble(data_x[x][y]);
				}
		for (int y=0; y<YDIM; y++)	
			for (int x=0; x<XDIM; x++)
				{
					b.addDouble(data_y[x][y]);
				}
		outDataPort->write();
	}
}

void flowViewer::velNorm(VelocityBuffer& data)
{

    double average, sptDer;
     int x, y;
     double vx, vy;
     int hx,hy, X,Y;

     double ax, bx, ay, by;
     double vvx, vvy;
     int vx_int, vy_int;

     int size;
     double norm, tmp;
     yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outImagePort-> prepare();
     img=normBaseImg;

     size = data.getSize();

     ax = (data.getVxMax() == 0 ? 1 : 40 / data.getVxMax());
     ay = (data.getVyMax() == 0 ? 1 : 40 / data.getVyMax());

     average = 0; sptDer = 0;
     for (int i = 0; i < size; ++i) {
         x = data.getX(i);
         y = data.getY(i);
         vx = data.getVx(i);
         vy = data.getVy(i);

         vvx = ax * vx;
         vvy = ax * vy;

         vx_int = (vvx >= 0 ? int(vvx+0.5) : int(vvx - 0.5) ) ;
         vy_int = (vvy >= 0 ? int(vvy+0.5) : int(vvy - 0.5) ) ;
         norm=sqrt(double(vx_int*vx_int+vy_int*vy_int));

//         norm=sqrt(vvx*vvx+vvy*vvy);
//        norm=sqrt(vx*vx+vy*vy) * 500;


        if (norm>= 0.0){

           img(x ,y ) = (norm * 5 > 255 ? 255 : int( norm + .5) * 5) ;
        }

         //average += int (norm + .5);
         average += norm ;
         //sptDer += int (norm + .5) * int (norm + .5);
         sptDer += norm * norm;
     }

     average = average / size;
     sptDer = sptDer / size;
     sptDer = sptDer - average * average;
     sptDer = sqrt(sptDer);
     cout << average << endl;
     //cout << average << " " << sptDer << endl;

     outImagePort->write();

}





void flowViewer::medianFilterSeprabale(VelocityBuffer & data){
    int size, idx;
    double x,y, vx, vy, mvx, mvy;
    double vxMax = -1, vyMax=-1;

    double ax [(2* MEDIAN_NGHBRHD +1) * (2* MEDIAN_NGHBRHD +1)];
    double ay [(2* MEDIAN_NGHBRHD +1) * (2* MEDIAN_NGHBRHD +1)];

    //resetVelMtrx();
    size = data.getSize();

    for (int i = 0; i < size; ++i) {
        x = data.getX(i) + MEDIAN_NGHBRHD;
        y = data.getY(i) + MEDIAN_NGHBRHD;

        xVels (x,y) = data.getVx(i);
        yVels (x,y) = data.getVy(i);
    }


    //for (int cntr = size-1; cntr >= 0; --cntr) {
    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);

        //Get the median Value of Velocities in the (x,y) neighborhood
        idx = 0;
        for (int i = x; i <= x+2*MEDIAN_NGHBRHD; ++i) {
            ax[idx] = xVels(i,y+MEDIAN_NGHBRHD);
            ay[idx++] = yVels(i,y+MEDIAN_NGHBRHD);
        }
        idx--;

        xVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = mvx = kth_smallest(ax, idx, idx/2);
        yVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = kth_smallest(ay, idx, idx/2);

        idx = 0;
        for (int j = y; j <= y+2*MEDIAN_NGHBRHD; ++j) {
             ax[idx] = xVels(x+MEDIAN_NGHBRHD,j);
             ay[idx++] = yVels(x+MEDIAN_NGHBRHD,j);
         }

        idx--;
        mvx = kth_smallest(ax, idx, idx/2);
        mvy = kth_smallest(ay, idx, idx/2);

        data.setVx(cntr, mvx);
        data.setVy(cntr, mvy);
        if (fabs(mvx) > vxMax)
            vxMax = fabs(mvx);
        if (fabs(mvy) > vyMax)
            vyMax = fabs(mvy);
    }


//cout << data.getVxMax() << " " << vxMax << " " << data.getVyMax() << " " << vyMax << endl;

//    data.setVxMax(vxMax);
    data.setVyMax(vyMax);

}


void flowViewer::medianFilter2D(VelocityBuffer & data){
    int size, idx;
    double x,y, vx, vy, mvx, mvy;
    double vxMax = -1, vyMax=-1;

    double ax [(2* MEDIAN_NGHBRHD +1) * (2* MEDIAN_NGHBRHD +1)];
    double ay [(2* MEDIAN_NGHBRHD +1) * (2* MEDIAN_NGHBRHD +1)];

    //resetVelMtrx();
    size = data.getSize();

    for (int i = 0; i < size; ++i) {
        x = data.getX(i) + MEDIAN_NGHBRHD;
        y = data.getY(i) + MEDIAN_NGHBRHD;

        xVels (x,y) = data.getVx(i);
        yVels (x,y) = data.getVy(i);
    }


    for (int cntr = 0; cntr < size; ++cntr) {
        x = data.getX(cntr);
        y = data.getY(cntr);

        //Get the median Value of Velocities in the (x,y) neighborhood
        idx = 0;
        for (int i = x; i <= x+2*MEDIAN_NGHBRHD; ++i) {
            for (int j = y; j <= y+2*MEDIAN_NGHBRHD; ++j) {
                ax[idx] = xVels(i,j);
                ay[idx++] = yVels(i,j);
            } // end-inner loop
        }// end-outer loop

        idx--;
        mvx = kth_smallest(ax, idx, idx/2);
        mvy = kth_smallest(ay, idx, idx/2);


        data.setVx(cntr, mvx);
        data.setVy(cntr, mvy);
        if (fabs(mvx) > vxMax)
            vxMax = fabs(mvx);
        if (fabs(mvy) > vyMax)
            vyMax = fabs(mvy);
    }



//cout << data.getVxMax() << " " << vxMax << " " << data.getVyMax() << " " << vyMax << endl;

    data.setVxMax(vxMax);
    data.setVyMax(vyMax);

}


double flowViewer::kth_smallest(double * a, int n, int k)
{
    double tmp, x;
    int i,j,l,m ;
    l=0 ; m=n-1 ;
    while (l<m) {
        x=a[k] ;
        i=l ;
        j=m ;
        do {
            while (a[i]<x) i++ ;
            while (x<a[j]) j-- ;
            if (i<=j) {
                //ELEM_SWAP(a[i],a[j]) ;
                tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
                i++ ; j-- ;
            }
        } while (i<=j) ;
        if (j<k) l=i ;
        if (k<i) m=j ;
    }
    return a[k] ;
}

