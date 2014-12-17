/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
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
#include "tsOptFlowViewer.hpp"
#include <iostream>
#define SCALE 50

using namespace std;
using namespace yarp::os;
tsOptFlowViewer::tsOptFlowViewer(std::string &_disp, double &_norm):displayMode(_disp), toNorm(_norm)
{
    createColorMap();
    mBaseImg.resize(10*XDIM,10*YDIM);
    yarp::sig::PixelRgb px;
    for(int x=0; x<10*XDIM; ++x)
    {
        for(int y=0; y<10*YDIM; ++y)
        {
            //mBaseImg(x,y)=160; //255;
            px.r=125; //255;
            px.g=125; //255;
            px.b=125; //255;
            mBaseImg.pixel(x, y)=px;
        }
    }

    xVels.resize(XDIM + 2*MEDIAN_NGHBRHD, YDIM + 2*MEDIAN_NGHBRHD);
    yVels.resize(XDIM + 2*MEDIAN_NGHBRHD, YDIM + 2*MEDIAN_NGHBRHD);

    exist=new bool[XDIM*YDIM];
    memset(exist, 0, XDIM*YDIM*sizeof(bool));

    zeroFlag = 0;

}

void tsOptFlowViewer::setOutPort(BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelRgb> > * oPort ){
    outPort = oPort;
}

tsOptFlowViewer::~tsOptFlowViewer()
{
    delete[] colorMap;
    delete[] exist;
}

double tsOptFlowViewer::kth_smallest(double * a, int n, int k)
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

void tsOptFlowViewer::resetVelMtrx(){
    zeroFlag++;
    if (zeroFlag == 3)
        zeroFlag = 0;

    if (zeroFlag == 0){
        xVels.zero();
        yVels.zero();
    }
}

void tsOptFlowViewer::medianFilterSeprabale(VelocityBuffer & data){
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


void tsOptFlowViewer::medianFilter2D(VelocityBuffer & data){
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

void tsOptFlowViewer::onRead(VelocityBuffer& data)
{

    int x, y;
    double vx, vy;
    int hx,hy, X,Y;

    double ax, bx, ay, by;
    double vvx, vvy;

    int size;
    double norm;
    double *ptr_norm;
    //yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=outPort-> prepare();
    yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=outPort-> prepare();
    img=mBaseImg;


//    medianFilter2D(data);
//    medianFilterSeprabale(data);

    size = data.getSize();
    ptr_norm=new double[size];
    memset(ptr_norm, 0, size*sizeof(double));

    //ax = (data.getVxMax() == 0 ? 1 : 20 / data.getVxMax());
    //ay = (data.getVyMax() == 0 ? 1 : 20 / data.getVyMax());

    double maxx=(data.getVxMax() == 0 ? 1 : data.getVxMax());
    double maxy=(data.getVyMax() == 0 ? 1 : data.getVyMax());
    double minx=data.getVxMin();
    double miny=data.getVyMin();
    double max=maxx>maxy?maxx:maxy;
    double min=minx<miny?minx:miny;

    memset(exist, 0, XDIM*YDIM*sizeof(bool));
    //for (int i = 0; i < size; ++i) {
    for (int i = (size-1); i >=0; --i)
    {
        x = data.getX(i);
        y = data.getY(i);
        if(!*(exist+x*YDIM+y))
        {
            *(exist+x*YDIM+y)=true;
            vx = data.getVx(i);
            vy = data.getVy(i);

    //        xVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = vx;
    //        yVels(x+MEDIAN_NGHBRHD, y+MEDIAN_NGHBRHD) = vy;

            //vvx = ax * vx;
            //vvy = ay * vy;
            //norm=vx*vx+vy*vy;
            norm=fabs(vx)+fabs(vy);
            ptr_norm[i]=norm;

    //        cout << norm << endl;
            if (norm>0.0)
            {
                X=5+10*x;
                Y=5+10*y;

    //            hx=(X+int(vvx+0.5))*SCALE;
    //            hy=(Y+int(vvy+0.5))*SCALE;
                //hx=X+(floor(sign(vx)*1+vx+sign(vx)*0.5))*SCALE;
                //hy=Y+(floor(sign(vy)*1+vy+sign(vy)*0.5))*SCALE;
                hx=X+floor( vx*toNorm +sign(vx)*0.5);
                hy=Y+floor( vy*toNorm +sign(vy)*0.5);

                //std::cout << "X: " << X << ", Y: " << Y << ", vx: " << vx << ", vy: " << vy << " => hx: " << hx << ", hy: " << hy << std::endl;
                if(!displayMode.compare("arrow"))
                {
                    //static const yarp::sig::PixelMono16 black=0;
                    static const yarp::sig::PixelRgb black(0,0,0);
                    //static const yarp::sig::PixelMono16 white=255;
                    static const yarp::sig::PixelRgb white(255, 255, 255);
                
                    yarp::sig::draw::addSegment(img,black,X,Y,hx,hy);
                    yarp::sig::draw::addCircle(img,black,hx,hy,2);
                }
                else if(!displayMode.compare("grayMap")) 
                {   
                    //vvx=floor( vx*1000 +sign(vx)*0.5);
                    //vvy=floor( vy*1000 +sign(vy)*0.5);
                    vvx=vx*1000;
                    vvy=vy*1000;
                    norm=(sqrt(vvx*vvx+vvy*vvy)/toNorm)*255;

                    //norm=(sqrt(vx*vx+vy*vy)/toNorm)*65535;

                    //norm=(sqrt(vx*vx+vy*vy)/toNorm)*16777216;
                    //std::cout << "min: " << min << ",max: " << max << ", vvx: " << vvx << ", vvy: " << vvy << ", norm: " << norm << std::endl;
                    yarp::sig::PixelRgb px;
                    //px.r=norm>16777216*3?16777216*3-3:(int)floor(norm+0.5)*3; 
                    //px.g=norm>16777216*3?16777216*3-2:(int)floor(norm+0.5)*3+1; 
                    //px.b=norm>16777216*3?16777216*3-1:(int)floor(norm+0.5)*3+2;
                    px.r=norm>255?255:(int)floor(norm+0.5); 
                    px.g=norm>255?255:(int)floor(norm+0.5); 
                    px.b=norm>255?255:(int)floor(norm+0.5);
                    for(int i=-5; i<=5; i++)
                        for(int ii=-5; ii<=5; ii++)
                        {
                            //img(X+i,Y+ii)=norm>65535?65535:(int)floor(norm+0.5);
                            img.pixel(X+i,Y+ii)=px;
                        }
                }
                if(!displayMode.compare("colorMap"))
                {   
                    //vvx=vx*1000;
                    //vvy=vy*1000;
                    //norm=(sqrt(vvx*vvx+vvy*vvy)/toNorm)*(16777216-3);
                    //norm=fabs(vx)+fabs(vy);

                    double nBins=6;
                    double al=1.2;
                    double step=toNorm/nBins;
                    uint bin=0;
                    for(double ii=step; ii<=toNorm; ii+=step)
                    {
                        if( norm>=al*log(1+ii-step) && norm<al*log(1+ii) )
                            break;
                        bin+=(uint)floor(1149.0/nBins);
                    }
                    //std::cout << "[tsOptFlowViewer] Norm: " << norm << ", bin: " << bin << std::endl;
                    if(bin>1149) bin=1149;
                    else if(bin<0) bin=0;

                    uchar r;
                    uchar g;
                    uchar b;

                    yarp::sig::PixelRgb px;
                    px.r=*(colorMap+bin);
                    px.g=*(colorMap+bin+1);
                    px.b=*(colorMap+bin+2);


                    //cout << "[tsOptFlowViewer] Retrieve the color in the computed colorMap" << endl;
                    //norm=norm>(16777216-3)?(16777216-3):norm;
                    //px.r=*(colorMap+(int)floor(norm+0.5)*3); 
                    //px.g=*(colorMap+(int)floor(norm+0.5)*3+1); 
                    //px.b=*(colorMap+(int)floor(norm+0.5)*3+2);
                    //std::cout << ", r: " << px.r << ", g: " << px.g << ", b: " << px.b << std::endl;
                    for(int i=-5; i<=5; i++)
                        for(int ii=-5; ii<=5; ii++)
                        {
                            //img(X+i,Y+ii)=norm>65535?65535:(int)floor(norm+0.5);
                            img.pixel(X+i,Y+ii)=px;
                        }
                }
            }
        }
    }
    if(!displayMode.compare("relativeColorMap"))
    {   
        double minNorm=gsl_stats_min(ptr_norm, 1, size);
        double maxNorm=gsl_stats_max(ptr_norm, 1, size);
        if(maxNorm-minNorm)
        {
        for(int in=0; in<size; ++in)
        {
            x = data.getX(in);
            y = data.getY(in);
            X=5+10*x;
            Y=5+10*y;
//            cout << "[tsOptFlowViewer] Current norm: " << ptr_norm[in] << endl;
//            cout << "[tsOptFlowViewer] Compute the index for color" << endl;
            norm=((ptr_norm[in]-minNorm)/(maxNorm-minNorm))*(16777216-3);
            std::cout << "min: " << minNorm << ",max: " << maxNorm << ", norm: " << norm; 
            yarp::sig::PixelRgb px;
//            cout << "[tsOptFlowViewer] Retrieve the color in the computed colorMap" << endl;
            px.r=*(colorMap+(int)floor(norm+0.5)*3); 
            px.g=*(colorMap+(int)floor(norm+0.5)*3+1); 
            px.b=*(colorMap+(int)floor(norm+0.5)*3+2);
            std::cout << ", r: " << px.r << ", g: " << px.g << ", b: " << px.b << std::endl;
            for(int i=-5; i<=5; i++)
                for(int ii=-5; ii<=5; ii++)
                {
                    //img(X+i,Y+ii)=norm>65535?65535:(int)floor(norm+0.5);
                    img.pixel(X+i,Y+ii)=px;
                }
        }
        }
    }
    delete[] ptr_norm;
    outPort->write();


}

double tsOptFlowViewer::sign(double &_in)
{
    if(_in<0)
        return -1;
    return 1;
}

void tsOptFlowViewer::createColorMap()
{
//    cout << "[tsOptFlowViewer] Create the color map..." << endl;
/*
    colorMap=new char[256*256*256*3];
    for(unsigned int r=0; r<256; r++)
        for(unsigned int g=0; g<256; g++)
            for(unsigned int b=0; b<256; b++)
            {
                *(colorMap+(((g+256*r)*256+b)*3)+0)=r;
                *(colorMap+(((g+256*r)*256+b)*3)+1)=g;
                *(colorMap+(((g+256*r)*256+b)*3)+2)=b;
            }
*/
//    cout << "[tsOptFlowViewer] Color map created" << endl;
    colorMap=new uchar[128*3*3];
    memset(colorMap, 0, 128*3*3);
    for(uint i=0; i<128; i++)
    {
        *(colorMap+(i*3))=255-i;
        *(colorMap+(i*3+1))=0;
        *(colorMap+(i*3+2))=0;

        *(colorMap+(i*3+128*3))=0;
        *(colorMap+(i*3+128*3+1))=255-i;
        *(colorMap+(i*3+128*3+2))=0;

        *(colorMap+(i*3+128*6))=0;
        *(colorMap+(i*3+128*6+1))=0;
        *(colorMap+(i*3+128*6+2))=255-i;
    }
}
