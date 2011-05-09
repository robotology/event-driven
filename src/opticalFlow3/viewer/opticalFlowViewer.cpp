#include "opticalFlowViewer.h"

opticalFlowViewer::opticalFlowViewer(double gain)
{
    mGain=gain;

    for (int x=-128; x<128; ++x)
    {
        for (int y=-128; y<128; ++y)
        {
            double alfa=1.5+1.5*atan2(double(y),double(x))/3.1415927;
            if (alfa>3.0) alfa=3.0; else if (alfa<0.0) alfa=0.0;
            double mag=2.0*sqrt(double(x*x+y*y));
            if (mag>255.0) mag=255.0;

            if (alfa<1.0)
            {
                double dA=(255.0-mag)+mag*alfa;
                double dB=(255.0-mag)+mag*(1.0-alfa);
                
                int a,b;
                if (dA<0.0) a=0; else if (dA>255.0) a=255; else a=(int)dA;
                if (dB<0.0) b=0; else if (dB>255.0) b=255; else b=(int)dB;
                
                mPalette[x+128][y+128]=yarp::sig::PixelRgb(a,b,int(255.0-mag));
            }
            else if (alfa<2.0)
            {
                alfa-=1.0;

                double dA=(255.0-mag)+mag*alfa;
                double dB=(255.0-mag)+mag*(1.0-alfa);
                
                int a,b;
                if (dA<0.0) a=0; else if (dA>255.0) a=255; else a=(int)dA;
                if (dB<0.0) b=0; else if (dB>255.0) b=255; else b=(int)dB;

                mPalette[x+128][y+128]=yarp::sig::PixelRgb(b,int(255.0-mag),a);
            }
            else
            {
                alfa-=2.0;

                double dA=(255.0-mag)+mag*alfa;
                double dB=(255.0-mag)+mag*(1.0-alfa);
                
                int a,b;
                if (dA<0.0) a=0; else if (dA>255.0) a=255; else a=(int)dA;
                if (dB<0.0) b=0; else if (dB>255.0) b=255; else b=(int)dB;

                mPalette[x+128][y+128]=yarp::sig::PixelRgb(int(255.0-mag),a,b);
            }
        }
    }

    for (int x=0; x<XDIM; ++x)
    {
        for (int y=0; y<YDIM; ++y)
        {
            mMapVx[x][y]=0.0;
            mMapVy[x][y]=0.0;
        }
    }

    mBaseImg.resize(XDIM,YDIM);
    
    for(int x=0; x<XDIM; ++x)
    {
        for(int y=0; y<YDIM; ++y)
        {
            //mBaseImg(x,y)=0xFF;
            mBaseImg(x,y)=yarp::sig::PixelRgb(255,255,255);
        }
    }

    mPort.open("/optflow/viewer:o");

    mTimeStart=yarp::os::Time::now();
}

opticalFlowViewer::~opticalFlowViewer()
{
}

void opticalFlowViewer::onRead(vecBuffer& data)
{
    mTimeCurrent=yarp::os::Time::now();
    double timeDiff=mTimeCurrent-mTimeStart;

    int x=data.get_x();
    int y=data.get_y();

    double vx=data.get_vx();
    double vy=data.get_vy();

    //printf("%d  %d  %lf  %lf\n",x,y,vx,vy);
    //fflush(stdout);
    
    mMapVx[x][y]+=vx;
    mMapVy[x][y]+=vy;
    double norm;

    if (timeDiff>=TNK_TIME)
    {
        mTimeStart=mTimeCurrent;
        //yarp::sig::ImageOf<yarp::sig::PixelMono>& img=mPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=mPort.prepare();
        img=mBaseImg;
        
        int px,py;

        for (int x=0; x<XDIM; ++x)
        {
            for (int y=0; y<YDIM; ++y)
            {
                vx=mMapVx[x][y];
                vy=mMapVy[x][y];

                if (vx!=0.0 || vy!=0.0)
                {
                    vx*=mGain;
                    vy*=mGain;
                    norm=vx*vx+vy*vy;

                    if (norm>=16384.0)
                    {
                        norm=127.9/sqrt(norm);
                        px=128+int(vx*norm);
                        py=128+int(vy*norm);
                    }
                    else
                    {
                        px=128+int(vx);
                        py=128+int(vy);
                    }

                    img(x,127-y)=mPalette[px][py];
                }

                mMapVx[x][y]=mMapVy[x][y]=0.0;
            }
        }
        
        mPort.write();
    }
}
