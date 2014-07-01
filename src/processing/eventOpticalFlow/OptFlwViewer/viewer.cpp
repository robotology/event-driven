#include "viewer.h"
#include <iostream>

viewer::viewer()
{
    mBaseImg.resize(10*XDIM,10*YDIM);
    
    for(int x=0; x<10*XDIM; ++x)
    {
        for(int y=0; y<10*YDIM; ++y)
        {
            mBaseImg(x,y)=160; //255;
        }
    }

    mPort.open("/image/opticalFlowFrame:o");

  //  mTimeStart=yarp::os::Time::now();
}

viewer::~viewer()
{
}

void viewer::onRead(VelocityBuffer& data)
{
   // mTimeCurrent=yarp::os::Time::now();
    //double timeDiff=mTimeCurrent-mTimeStart;
    int x, y;
    double vx, vy;
    int hx,hy, X,Y;

    double vxRng, vyRng;
    double ax, bx, ay, by;
    double vvx, vvy;

    int size;
    double norm;
    yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=mPort.prepare();
    img=mBaseImg;

    size = data.getSize();
   
    vxRng = data.getVxMax() - data.getVxMin();
    ax = 50 / data.getVxMax();
    vyRng = data.getVyMax() - data.getVyMin();
    ay = 50 / data.getVyMax();

    //printf("%d  %lf  %lf\n",size,ax ,ay);

    for (int i = 0; i < size; ++i) {
        x = data.getX(i);
        y = data.getY(i);
        vx = data.getVx(i);
        vy = data.getVy(i);

        vvx = ax * vx; //vvx = ax * abs(vx) + bx;//
        vvy = ay * vy; //vvy = ay * abs(vy) + by; //

       // norm=vx*vx+vy*vy;
       // if (norm>0.0)
       // {
            //std::cout << x << " " << y << " " << vx << " " << vy << std::endl;
            X=5+10*x;
            Y=5+10*y;
        //    norm=50.0/sqrt(norm);
            //norm = 40000000;

            hx=X+int(vvx+0.5); //hx=X+int(norm*vx+0.5);//
            hy=Y+int(vvy+0.5); //hy=Y+int(norm*vy+0.5);//

            static const yarp::sig::PixelMono16 black=0;
            static const yarp::sig::PixelMono16 white=255;
            //if (hx > X){
                yarp::sig::draw::addSegment(img,black,X,Y,hx,hy);
                yarp::sig::draw::addCircle(img,black,hx,hy,2);
            /*}
            else{
                yarp::sig::draw::addSegment(img,white,X,Y,hx,hy);
                yarp::sig::draw::addCircle(img,white,hx,hy,2);
            }*/
     //   }

    }


    //printf("%d  %d  %lf  %lf\n",x,y,vx,vy);
    //fflush(stdout);
    

    //mBaseImg = img;
        mPort.write();
   //     yarp::os::Time::delay(1);
    //    mTimeStart=yarp::os::Time::now();

}
