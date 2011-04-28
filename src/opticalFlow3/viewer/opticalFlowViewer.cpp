#include "opticalFlowViewer.h"

opticalFlowViewer::opticalFlowViewer(bool color)
{
    mColor=color;

    for (int x=-32; x<=32; ++x)
    {
        for (int y=-32; y<=32; ++y)
        {
            double alfa=1.5+1.5*atan2(double(y),double(x))/3.1415927;
            if (alfa>3.0) alfa=3.0; else if (alfa<0.0) alfa=0.0;

            if (alfa<1.0)
            {
                mPalette[x+32][y+32]=yarp::sig::PixelRgb((unsigned char)(255.0*alfa),(unsigned char)(255.0*(1.0-alfa)),0);
            }
            else if (alfa<2.0)
            {
                alfa-=1.0;
                mPalette[x+32][y+32]=yarp::sig::PixelRgb((unsigned char)(255.0*(1.0-alfa)),0,(unsigned char)(255.0*alfa));
            }
            else
            {
                alfa-=2.0;
                mPalette[x+32][y+32]=yarp::sig::PixelRgb(0,(unsigned char)(255.0*alfa),(unsigned char)(255.0*(1.0-alfa)));
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

    mBaseImg.resize(4*XDIM,4*YDIM);
    
    for(int x=0; x<4*XDIM; ++x)
    {
        for(int y=0; y<4*YDIM; ++y)
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

    int hx,hy;

    if (timeDiff>=TNK_TIME)
    {
        //yarp::sig::ImageOf<yarp::sig::PixelMono>& img=mPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=mPort.prepare();
        img=mBaseImg;
        
        int X,Y;
        int px,py;

        for (int x=0; x<XDIM; ++x)
        {
            for (int y=0; y<YDIM; ++y)
            {
                vx=mMapVx[x][y];
                vy=mMapVy[x][y];

                if ((norm=vx*vx+vy*vy)>0.0)
                {
                    X=2+4*x;        
                    Y=2+4*y;
                    
                    hx=X+int(vx);
                    hy=511-Y-int(vy);

                    if (mColor)
                    {
                        if (norm>1024.0)
                        {
                            norm=32.0/sqrt(norm);
                            px=32+int(vx*norm);
                            py=32+int(vy*norm);
                        }
                        else
                        {
                            px=32+int(vx);
                            py=32+int(vy);
                        }

                        yarp::sig::draw::addSegment(img,mPalette[px][py],X,511-Y,hx,hy);
                        yarp::sig::draw::addCircle(img,mPalette[px][py],hx,hy,2);
                    }
                    else
                    {
                        //static const yarp::sig::PixelMono black=0;
                        static const yarp::sig::PixelRgb black(0,0,0);
                        yarp::sig::draw::addSegment(img,black,X,511-Y,hx,hy);
                        yarp::sig::draw::addCircle(img,black,hx,hy,2);
                    }
                }

                mMapVx[x][y]=mMapVy[x][y]=0.0;
            }
        }
        
        mPort.write();
        
        mTimeStart=yarp::os::Time::now();
    }
}
