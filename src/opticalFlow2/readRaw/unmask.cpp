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

void Unmask::unmaskData(unsigned int* buffer,int size)
{
    size/=4;

    int x;
    int y;
    int camera;
    
    //int polarity;
    //unsigned long timeStamp;
    
    int blob;

    for (int j=0; j<size; j+=2)
    {
        blob=(int)buffer[j];

        camera=(0x8000 & blob)>>15;

        if (camera==mChannel || mChannel==-1)
        {
            //y=(0x7F00 & blob)>>8;
            //x=(0x00FE & blob)>>1;
            
            x=127-((0x7F00 & blob)>>8);
            y=127-((0x00FE & blob)>>1);

            if (x>=5 && y>=5 && x<=122 && y<=122)
            {
                //polarity=0x0001 & blob;
                //timeStamp=buffer[j+1];

                objSynapse->filter(x,y,(0x0001 & blob),buffer[j+1]);
            }
        }
    }
}
