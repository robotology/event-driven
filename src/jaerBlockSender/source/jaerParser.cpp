#include "jaerParser.hpp"

using namespace std;
jaerParser::jaerParser()
{
    timestampMonotonyWrap = 0;

    bodySize=0;
    jaerSizeType=0;
    currentEvent=0;

    buffer=NULL;
}

jaerParser::jaerParser(string& jaerFileName)
{
    this->parse(jaerFileName);
}

jaerParser::jaerParser(const jaerParser& _in)
{
    bodySize=_in.bodySize;
    jaerSizeType=_in.jaerSizeType;
    currentEvent=_in.currentEvent;
    timestampMonotonyWrap=_in.timestampMonotonyWrap;
    delete buffer;
    buffer=new char[bodySize];
    memcpy(buffer, _in.buffer, bodySize);
}


jaerParser::~jaerParser(){}

jaerParser& jaerParser::operator=(const jaerParser& _in)
{
    if(&_in!=this)
    {
        bodySize=_in.bodySize;
        jaerSizeType=_in.jaerSizeType;
        currentEvent=_in.currentEvent;
        timestampMonotonyWrap=_in.timestampMonotonyWrap;
        delete buffer;
        buffer=new char[bodySize];
        memcpy(buffer, _in.buffer, bodySize);
    }
    return *this;
}

int jaerParser::parse(string& jaerFileName)
{
    timestampMonotonyWrap=0;
    currentEvent=0;
cout << "Open jAER file: " << jaerFileName << endl;
    FILE* fJAER = fopen(jaerFileName.c_str(), "rb");
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
	return 0;
}
int jaerParser::read(unsigned int& addrx, unsigned int& addry , int& polarity, unsigned int& timestamp)
{
    int res=1;
    if( (jaerSizeType==4) && ((buffer[currentEvent+3]&0x80)==0x80) )
    {
        // timestamp bit 15 is one -> wrap
        // now we need to increment the wrapAdd
        timestampMonotonyWrap+=0x4000; //uses only 14 bit timestamps
    }
    else if( (jaerSizeType==4) && ((buffer[currentEvent+3]&0x40)==0x40  ) )
    {
        // timestamp bit 14 is one -> wrapAdd reset
        // this firmware version uses reset events to reset timestamps
        timestampMonotonyWrap=0;
    }
    else
    {
//----------------------------Unmask the data----------------------------------------------------------//
        unsigned int blob;
        switch(jaerSizeType)
        {
            case 4: blob = ((0x000000FF&buffer[currentEvent]))|((0x000000FF&buffer[currentEvent+1])<<8);
                    unmaskEvent(blob, addrx, addry, polarity);
                    timestamp = (((0x000000FF&buffer[currentEvent+2]))|((0x000000FF&buffer[currentEvent+3])<<8))+timestampMonotonyWrap;
                    break;
            case 6: blob = ((0x000000FF&buffer[currentEvent]))|((0x000000FF&buffer[currentEvent+1])<<8);
                    unmaskEvent(blob, addrx, addry, polarity);
                    timestamp = ((0x000000FF&buffer[currentEvent+5]))|((0x000000FF&buffer[currentEvent+4])<<8)|((0x000000FF&buffer[currentEvent+3])<<16)|((0x000000FF&buffer[currentEvent+2])<<24);
                    break;
            case 8: blob = (0x000000FF&buffer[currentEvent+3])|((0x000000FF&buffer[currentEvent+2])<<8)|((0x000000FF&buffer[currentEvent+1])<<16)|((0x000000FF&buffer[currentEvent])<<24);
                    unmaskEvent(blob, addrx, addry, polarity);
                    timestamp = (0x000000FF&buffer[currentEvent+7])|((0x000000FF&buffer[currentEvent+6])<<8)|((0x000000FF&buffer[currentEvent+5])<<16)|((0x000000FF&buffer[currentEvent+4])<<24);
                    break;
        }
        if(blob&0x8000)
            res=2;
    }
    currentEvent+=jaerSizeType;
    if(currentEvent==bodySize-jaerSizeType)
    {
        currentEvent=0;
        return 0;
    }
    else
        return res;
}
void jaerParser::unmaskEvent(unsigned int rawEvent, unsigned int& x, unsigned int& y, int& pol)
{
    x   = (rawEvent & 0x000000fE)>>1;
	y   = (rawEvent & 0x00007f00)>>8;
	pol = (((rawEvent & 0x00000001)>>0)==0)?-1:1;	//+1 ON, -1 OFF
}
