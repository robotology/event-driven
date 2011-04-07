#include "unmask.h"

#include <yarp/os/Time.h>

Unmask::Unmask(int channel)
{
    mChannel=channel;
	mWrapAdd=0;

    objSynapse=new Synapse(
        128,   // x dim
        128,   // y dim
        2,     // neighborhood
        0.8,   // sigma
        2.0,   // threshold
        1.0,   // alpha // 5.0
        250.0, // tau
        5000   // accTime
    );

    std::string portName("/image/opticalFlow:o");
    objSynapse->openPort(portName);
}

Unmask::~Unmask()
{
    objSynapse->closePort();
    delete objSynapse; 
}

void Unmask::unmaskData(unsigned char* buffer,int size,int type)
{
    int x;
    int y;
    int polarity;
    int timeStamp;
    int camera;

    if (type==4)
    {
        for (int j=0; j<size; j+=type)
        {
            if (buffer[j+3]&0x80)
            {
                mWrapAdd+=0x4000;
            }
            else if (buffer[j+3]&0x40)
            {
                mWrapAdd=0;
            }
            else
            {
                x=buffer[j]>>1;
                y=buffer[j+1]&0x7F;
                
                if (x>=5 && y>=5 && x<=122 && y<=122)
                {
                    polarity=(buffer[j]&0x01)?1:-1;
                    timeStamp=((buffer[j+3]<<8)|buffer[j+2])+mWrapAdd;

                    objSynapse->filter(x,y,polarity,timeStamp);
                }
            }
        }

        return;
    }

    if (type==6)
    {
        for (int j=0; j<size; j+=type)
        {
            x=buffer[j+1]>>1;
            y=buffer[j]&0x7F;
            
            if (x>=5 && y>=5 && x<=122 && y<=122)
            {
                polarity=(buffer[j+1]&0x01)?1:-1;
                timeStamp=(buffer[j+2]<<24)|(buffer[j+3]<<16)|(buffer[j+4]<<8)|buffer[j+5];

                objSynapse->filter(x,y,polarity,timeStamp);
            }
        }

        return;
    }

    if (type==8)
    {
        for (int j=0; j<size; j+=type)
        {
            x=buffer[j+3]>>1;
            y=127-(buffer[j+2]&0x7F);
            camera=buffer[j+2]&0x80;

            if (camera==mChannel)
            {
                if (x>=5 && y>=5 && x<=122 && y<=122)
                {
                    polarity=(buffer[j+3]&0x01)?1:-1;
                    timeStamp=(buffer[j+4]<<24)|(buffer[j+5]<<16)|(buffer[j+6]<<8)|buffer[j+7];

                    objSynapse->filter(x,y,polarity,timeStamp);

                    //static double timeStartClock=yarp::os::Time::now();
                    //static double timeStartStamp=1E-6*double(timeStamp);

                    //double timeClockNow=yarp::os::Time::now()-timeStartClock;
                    //double timeStampNow=1E-6*double(timeStamp)-timeStartStamp;

                    //if (timeStampNow>timeClockNow) yarp::os::Time::delay(timeStampNow-timeClockNow);
                }
            }
        }
    }
}
