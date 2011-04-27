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

    std::string portName("/optflow/vectors:o");
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

        if (camera==mChannel)
        {
            //y=(0x7F00 & blob)>>8;
            //x=(0x00FE & blob)>>1;
            
            x=127-((0x7F00 & blob)>>8);
            y=127-((0x00FE & blob)>>1);

            if (x>=0 && y>=0 && x<=127 && y<=127)
            {
                //polarity=0x0001 & blob;
                //timeStamp=buffer[j+1];

                objSynapse->filter(x,y,(0x0001 & blob),buffer[j+1]);
            }
        }
    }
}

void Unmask::unmaskFile(const char* fileName)
{
    int x;
    int y;
    double p;
    unsigned int t;

    FILE* data=fopen(fileName,"rb");
    fseek(data,0,SEEK_END);
    int size=ftell(data);
    fseek(data,0,SEEK_SET);
    size/=20;

    for (int j=0; j<size; ++j)
    {
        fread(&x,sizeof(int),1,data);
        fread(&y,sizeof(int),1,data);
        fread(&p,sizeof(double),1,data);
        fread(&t,sizeof(unsigned int),1,data);

        static unsigned int tStart=t;
        static double timeStart=yarp::os::Time::now();
        
        if (x>=0 && y>=0 && x<=127 && y<=127)
        {
            objSynapse->filter(x,y,p>0.0?1:0,t);
        }

        double tLocal=0.000001*double(t-tStart);
        double timeLocal=yarp::os::Time::now()-timeStart;

        if (tLocal>timeLocal)
        {
            //yarp::os::Time::delay(tLocal-timeLocal);
        }
    }
}