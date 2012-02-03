#include "flowViewer.h"
#include <iostream>

flowViewer::flowViewer()
{

    mBaseImg.resize(10*XDIM,10*YDIM);
    
    for(int x=0; x<10*XDIM; ++x)
    {
        for(int y=0; y<10*YDIM; ++y)
        {
            mBaseImg(x,y)=160; //255;
        }
    }

    xVels.resize(XDIM + 2*MEDIAN_NGHBRHD, YDIM + 2*MEDIAN_NGHBRHD);
    yVels.resize(XDIM + 2*MEDIAN_NGHBRHD, YDIM + 2*MEDIAN_NGHBRHD);


    zeroFlag = 0;

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
    if (zeroFlag == 2)
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
    double norm;
    yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outPort-> prepare();
    img=mBaseImg;


    medianFilter2D(data);
//    medianFilterSeprabale(data);

    size = data.getSize();
   

    ax = 20 / data.getVxMax();
    ay = 20 / data.getVyMax();

    //printf("%d  %lf  %lf\n",size,ax ,ay);

    for (int i = 0; i < size; ++i) {
        x = data.getX(i);
        y = data.getY(i);
        vx = data.getVx(i);
        vy = data.getVy(i);

//        xVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = vx;
//        yVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = vy;

        vvx = ax * vx; //vvx = ax * abs(vx) + bx;//
        vvy = ay * vy; //vvy = ay * abs(vy) + by; //

       // norm=vx*vx+vy*vy;
       // if (norm>0.0)
//        if (vvx != 0 || vvy != 0)
//        {
            //std::cout << x << " " << y << " " << vx << " " << vy << std::endl;
            X=5+10*x;
            Y=5+10*y;
        //    norm=50.0/sqrt(norm);
            //norm = 40000000;

            hx=X+int(vvx+0.5); //hx=X+int(norm*vx+0.5);//
            hy=Y+int(vvy+0.5); //hy=Y+int(norm*vy+0.5);//

            static const yarp::sig::PixelMono16 black=0;
            static const yarp::sig::PixelMono16 white=255;

            yarp::sig::draw::addSegment(img,black,X,Y,hx,hy);
            yarp::sig::draw::addCircle(img,black,hx,hy,2);

//        }

    }


    //printf("%d  %d  %lf  %lf\n",x,y,vx,vy);
    //fflush(stdout);
    

    //mBaseImg = img;
    outPort->write();


}
