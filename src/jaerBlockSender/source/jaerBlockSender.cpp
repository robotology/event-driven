#include "jaerBlockSender.hpp"

using namespace std;
using namespace yarp::os;
using namespace emorph::ebuffer;
using namespace emorph::eunmask;

jaerBlockSender::jaerBlockSender()
{
    bodySize=0;
    jaerSizeType=0;
    currentEvent=0;

    buffer=NULL;

    outputPort.open("/jaerBlockSender:o");
}

jaerBlockSender::~jaerBlockSender()
{
    delete buffer;
    outputPort.close();
}

int jaerBlockSender::load(string jaerFileName)
{
    currentEvent=0;
    cout << "Open jAER file: " << jaerFileName << endl;
    FILE* fJAER = fopen(jaerFileName.c_str(), "rb");
    if(fJAER==NULL)
    {
        cerr << "Error -1: File not found" << endl;
        return -1;
    }
    int startBody=0;

    char line[100];
    fgets(line, 100, fJAER);

    char currentLineHead[10];
    strcpy(currentLineHead, "#!AER-DAT");
    float version=0;
    while(line[0]=='#')
    {
        //Get the version of the jAER file
        if (strncmp(line,currentLineHead, 5)==0)
            sscanf(line,"%*9s%f", &version);
        //Update the position where the body begin
        startBody=ftell(fJAER);
        //gets the current line including line
        fgets(line, 100, fJAER);
    }
    fseek(fJAER, 0, SEEK_END);
    bodySize=ftell(fJAER) - startBody;
    cout << "size of the body: " << bodySize << endl;
    fseek(fJAER, startBody, SEEK_SET);
    delete buffer;
    buffer = new char[bodySize];
    int res = fread(buffer, 1, bodySize, fJAER);
    switch((int)version)
    {
        case 0:     printf("No #!AER-DAT version header found, assuming 16 bit addresses\n");
                    jaerSizeType=6;
                    break;
        case 1:     printf("Addresses are 16 bit\n");
                    jaerSizeType=6;
                    break;
        case 2:     printf("Addresses are 32 bit\n");
                    jaerSizeType=8;
                    break;
        default:    printf("Unknown file version %f\n",version);
                    jaerSizeType=6;
                    break;
    }
cout << "type of file: " << jaerSizeType << endl;
    fclose(fJAER);
    index=0;
    return 0;
}

void jaerBlockSender::send(int _sz)
{
    printf("jaerBlockSender send\n");
    if(_sz==-1)
        _sz=bodySize;
    else if(index+_sz>bodySize)
        _sz=bodySize-index;
    printf("\tAddress of the buffer: %x, address with index: %x, size of the packet: %d, number of events expected: %d\n", buffer, buffer+index, _sz, _sz/jaerSizeType);
    if(_sz)
    {
        char *tmpBuf=new char[_sz];
        memcpy(tmpBuf, buffer+index, _sz);
        eventBuffer outputBuffer(tmpBuf, _sz);
        eventBuffer& tmp = outputPort.prepare();
        tmp = outputBuffer;
        outputPort.write();
        index+=_sz;
        delete[] tmpBuf;
    }
}

void jaerBlockSender::prepareData(uint _dt)
{
    cout << "[jaerBlockSender] Prepare data, block of " << _dt << "ms" << endl;

    cout << "[jaerBlockSender] Instantiate unmaskDVS128, type of data " << jaerSizeType << endl;
    eventUnmaskDVS128 unmaskor(jaerSizeType);
    cout << "[jaerBlockSender] Set the buffer of i-unmaskDVS128, size of the buffer: " << bodySize << endl;
    unmaskor.setBuffer(buffer, bodySize);

    uint addrx, addry;
    int polarity;
    uint eye;
    uint refts, timestamp;
    indexes.clear();
    int res;
    cout << "[jaerBlockSender] Begining of the preparation" << endl;
    do
    {
        res=unmaskor.getUmaskedData(addrx, addry, polarity, eye, refts);
    }while(!res);
    indexes.push_back(0);
    while(res)
    {
        res=unmaskor.getUmaskedData(addrx, addry, polarity, eye, timestamp);
        if(res && (refts+_dt<timestamp))
        {
            refts=timestamp;
            //indexes.push_back(unmaskor.getIndex()-jaerSizeType);
            indexes.push_back(unmaskor.getIndex());
        }
    }
    //indexes.push_back(unmaskor.getIndex()-jaerSizeType);
    cout << "[jaerBlockSender] Number of indexes: " << indexes.size() << endl;
}

int jaerBlockSender::sendBlocks(uint _from, uint _to, uint _dt)
{
    struct timespec start, end;
    if(indexes.empty())
        return -2;

    if(_from>indexes.size()-2)
        return -1;
    if(_to>indexes.size()-2 || _to==-1)
        _to=indexes.size()-2;
    for(uint i=0; i<=_to; i++)
    {
        send(indexes[i+1]-indexes[i]);
        clock_gettime(CLOCK_MONOTONIC, &start);
        do
        {
            clock_gettime(CLOCK_MONOTONIC, &end);
        }while((end.tv_sec*1E9+end.tv_nsec)-(start.tv_sec*1E9+start.tv_nsec)<_dt*1E3);
    }
    return 1;
}
