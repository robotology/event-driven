#include "flowViewer.h"
#include <iostream>

flowViewer::flowViewer()
{

    mBaseImg.resize(4*XDIM,4*YDIM);
    
    for(int x=0; x<4*XDIM; ++x)
    {
        for(int y=0; y<4*YDIM; ++y)
        {
//            if (x < 4 * 69)
               mBaseImg(x,y)=160; //255;
//            else
//               mBaseImg(x,y)=100; //255;
        }
    }

    normBaseImg.resize(XDIM, YDIM);
    for(int x=0; x<XDIM; ++x)
    {
       for(int y=0; y<YDIM; ++y)
       {
              normBaseImg(x,y)=100; //255;
       }
    }

//    xVels.resize(XDIM + 2*MEDIAN_NGHBRHD, YDIM + 2*MEDIAN_NGHBRHD);
//    yVels.resize(XDIM + 2*MEDIAN_NGHBRHD, YDIM + 2*MEDIAN_NGHBRHD);

    xVels.resize(XDIM, YDIM);
    xVels.zero();
    yVels.resize(XDIM, YDIM);
    yVels.zero();
    timeRec.resize(XDIM, YDIM);
    timeRec.zero();


    zeroFlag = 0;
    cntrR = 0; cntrL = 0;
    velR = 0; velL = 0;


}

void flowViewer::setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > * oPort ){
    outPort = oPort;
}

flowViewer::~flowViewer()
{
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

void flowViewer::resetVelMtrx(){
    zeroFlag++;
    if (zeroFlag == 3)
        zeroFlag = 0;

    if (zeroFlag == 0){
        xVels.zero();
        yVels.zero();
    }
}

void flowViewer::medianFilterSeprabale(VelocityBuffer & data){
    int size, idx;
    double x,y, vx, vy, mvx, mvy;
    double vxMax = -1, vyMax=-1;

    double ax [(2* MEDIAN_NGHBRHD +1) * (2* MEDIAN_NGHBRHD +1)];
    double ay [(2* MEDIAN_NGHBRHD +1) * (2* MEDIAN_NGHBRHD +1)];

    resetVelMtrx();
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

    resetVelMtrx();
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

void flowViewer::run(VelocityBuffer& data)
{

    int x, y;
    double vx, vy;
    int hx,hy, X,Y;

    double ax, bx, ay, by;
    double vvx, vvy;

    int size;
    double norm, tmp;
    yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outPort-> prepare();
    img=mBaseImg;

velL =  velR = 0 ;
cntrL = cntrR = 0;


//    medianFilter2D(data);
//    medianFilterSeprabale(data);

    size = data.getSize();
   
    ax = (data.getVxMax() == 0 ? 1 : 40 / data.getVxMax());
    ay = (data.getVyMax() == 0 ? 1 : 40 / data.getVyMax());
    cout << ax << " " << ay << endl;
    for (int i = 0; i < size; ++i) {
        x = data.getX(i);
        y = data.getY(i);
        vx = data.getVx(i);
        vy = data.getVy(i);

//        xVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = vx;
//        yVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = vy;
//        cout << vx << " " << vy << " " << vvx << " "  << vvy << endl;

        vvx = ax * vx;
        vvy = ay * vy;

        norm=vvx*vvx+vvy*vvy;
//
//        if (x < 69 ){
//            cntrL++;
//            velL += norm;
//        }else{
//            cntrR++;
//            velR += norm;
//        }


        //cout << data.getRel(i) << endl;
        if (norm>0.0){

            X=2+4*x;
            Y=2+4*y;

            hx=X+ (vvx >= 0 ? int(vvx+0.5) : int(vvx - 0.5) ) ;
            hy=Y+ (vvy >= 0 ? int(vvy+0.5) : int(vvy - 0.5) ) ;

            static const yarp::sig::PixelMono16 black=0;
            static const yarp::sig::PixelMono16 white=255;

            yarp::sig::draw::addSegment(img,black,X,Y,hx,hy);
            yarp::sig::draw::addCircle(img,black,hx,hy,2);

        }

    }

//    avgL = velL / cntrL;
//    avgR = velR / cntrR;
//    cout << avgL << " " << avgR << endl;

    outPort->write();

}

void flowViewer::velNorm(VelocityBuffer& data)
{

    int x, y;
    double vx, vy;
    int hx,hy, X,Y;

    double ax, bx, ay, by;
    double vvx, vvy;



    int size;
    double norm, tmp;
    yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outPort-> prepare();
    img=normBaseImg;


    size = data.getSize();
//    if (size < 20)
//        return;

    ax = (data.getVxMax() == 0 ? 1 : 1 / data.getVxMax());
    ay = (data.getVyMax() == 0 ? 1 : 1 / data.getVyMax());

    tmp = 0;
    double sptDer = 0;
    for (int i = 0; i < size; ++i) {
        x = data.getX(i);
        y = data.getY(i);
        vx = data.getVx(i);
        vy = data.getVy(i);

        vvx = ax * vx;
        vvy = ay * vy;
        norm=  int( sqrt(vvx*vvx+vvy*vvy) * 100 + .5);//  sqrt(vvy*vvy);
//        norm = (vvx >= 0 ? int(vvx+0.5)*int(vvx+0.5) : int(vvx - 0.5)*int(vvx-0.5) ) ;
//        norm += (vvy >= 0 ? int(vvy+0.5)*int(vvy+0.5) : int(vvy - 0.5)*int(vvy-0.5) ) ;
//        norm = sqrt(norm);


        if (x >= 1 && y>= 1 && x < 127 && y < 127) {
            for (int j = -1; j <= 1; ++j) {
                for (int k = -1; k <= 1; ++k) {
                    img(x + j,y + k) = (norm * 5 > 255 ? 255 : norm * 5) ;
                }
            }
        }
        else
           img(x,y) = (norm * 255 > 255 ? 255 : norm * 255) ;

        cout << norm << endl;

        tmp += norm;
        sptDer += norm * norm;
    }



    tmp = tmp / size;
    sptDer = sptDer / size;
    sptDer = sptDer - tmp * tmp;
//    cout << tmp << " " << sptDer << endl;

    outPort->write();

}




