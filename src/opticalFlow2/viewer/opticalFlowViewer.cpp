#include "opticalFlowViewer.h"

opticalFlowViewer::opticalFlowViewer()
{
    mVx.resize(WIDTH,HEIGHT);
    mVy.resize(WIDTH,HEIGHT);
    mVx.zero();
    mVy.zero();

    indQuiv=0;

    mBlack=0;

    mBaseImg.resize(4*WIDTH,4*HEIGHT);
    
    for(int x=0; x<4*WIDTH; ++x)
    {
        for(int y=0; y<4*HEIGHT; ++y)
        {
            mBaseImg(x,y)=128;
        }
    }

    mPort.open("/image/opticalFlowFrame:o");

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
    
    if (vx!=0.0 || vy!=0.0)
    {
        double norm=15.0/std::sqrt(vx*vx+vy*vy);
        mVx(x,y)+=norm*vx;
        mVy(x,y)+=norm*vy;
    }
    /*
    else
    {
        mVx(x,y)=mVy(x,y)=0.0;
    }
    */

    if (timeDiff>=TNK_TIME)
    {
        yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=mPort.prepare();
        img=mBaseImg;
        
        int X,Y;

        for (int x=0; x<WIDTH; ++x)
        {
            for (int y=0; y<HEIGHT; ++y)
            {
                if (mVx(x,y)!=0.0 || mVy(x,y)!=0.0)
                {
                    X=2+4*x;
                    Y=2+4*y;

                    /*
                    //Y=2+4*(HEIGHT-1-y);
                    X=4*x;
                    Y=4*y;
                    for (int XX=X; XX<X+4; ++XX)
                        for (int YY=Y; YY<Y+4; ++YY)
                            img(XX,YY)=128+int(32.0*mVx(x,y));
                    */

                    //img(X,Y)=mBlack;
                    yarp::sig::draw::addSegment(img,mBlack,X,Y,X+int(mVx(x,y)+0.5),Y+int(mVy(x,y)+0.5));
                }
            }
        }
        
        mPort.write();

        mVx.zero();
        mVy.zero();
        
        mTimeStart=yarp::os::Time::now();
    }
}
