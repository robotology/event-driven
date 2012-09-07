#include "convert.hpp"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

using namespace emorph::eunmask;

convert::convert(unsigned int i_width, unsigned int i_height, unsigned int _acc, string _src, unsigned int _type, unsigned int _ori)
//:RateThread(THRATE), accTime(_acc)
:accTime(_acc), orientation(_ori)
{
    base_img.resize(i_width, i_height);
    for(int i=0; i<i_width; i++)
    {
        for(int j=0; j<i_height; j++)
        {
            base_img(i,j) = 125;
        }
    }
    height = i_height;
    width = i_width;

    portLeft.open("/image/cfLeft:o");
    portRight.open("/image/cfRight:o");

    startNewFrame=true;

    if(!_src.compare("icub"))
        unmasker=new eventUnmaskICUB();
    else if(!_src.compare("icubcircbuf"))
        unmasker=new eventUnmaskICUBcircBuf();
    else if(!_src.compare("dvs"))
        unmasker=new eventUnmaskDVS128(_type);
}
convert::~convert()
{
    portLeft.close();
    portRight.close();
}

void convert::setBuffer(char* _buf, unsigned int _sz)
{
#ifdef _DEBUG
    cout << "[convert] throw buffer to unmask instance" << endl;
#endif
    unmasker->setBuffer(_buf, _sz);
}

void convert::initFrames()
{
    //time(&stime);
    //startNewFrame=false;
    imgLeft = base_img;
    imgRight = base_img;
    int res;
    do
    {
        res=unmasker->getUmaskedData(addrx, addry, polarity, eye, refts);
    }while(!res);
    if(orientation)
        flip();
    if(eye==0)
        imgLeft(addrx, addry)+=CONTRAST*polarity;
    else
        imgRight(addrx, addry)+=CONTRAST*polarity;
}

void convert::createFrames()
{
    while(1)
    {
        if(startNewFrame)
        {
            initFrames();
            startNewFrame=false;
        }
        int res=unmasker->getUmaskedData(addrx, addry, polarity, eye, timestamp);
        //if(orientation)
        //    flip();
            //time(&ctime);
            //cout << "\t- diff real time (ms): " << difftime(ctime, stime)*1000 << ", diff stamped time (ms): " << timestamp-refts << endl;
        if(res)
        {
            cout << "\taddrx: " << addrx << ", addry: " << addry << ", polarity: " << polarity << ", eye: " << eye << ", timestamp: " << timestamp << endl;
#ifdef _DEBUG
            cout << "[convert] refts = " << refts << ", timestamp = " << timestamp << ", diff = " << timestamp-refts << endl;
#endif
            if(refts+accTime>timestamp)
            {
                if(eye==0)
                    imgLeft(addrx, addry)+=CONTRAST*polarity;
                else
                    imgRight(addrx, addry)+=CONTRAST*polarity;
            }
            else
            {
                sendFrames(imgLeft, portLeft);
                sendFrames(imgRight, portRight);
                startNewFrame=true;
            }
        }
    }
}

void convert::sendFrames(ImageOf<PixelMono16>& i_img, BufferedPort<ImageOf<PixelMono16> >& _port)
{
#ifdef _DEBUG
    std::cerr << "[convert] Send images" << std::endl;
#endif
    ImageOf<PixelMono16>& tmp = _port.prepare();
    tmp = i_img;
    _port.write();
}

void convert::run()
{
    while(1)
        createFrames();
}

int convert::sign(int i_val)
{
    if(i_val > 0)
        return 1;
    else if(i_val < 0)
        return -1;
    else
        return 0;

}

float convert::mean_event(int i_nE)
{
    static float number_of_data = 0.0;
    static float mean_time_loop = 0.0;
    number_of_data++;
    if(number_of_data == 1)
        mean_time_loop = (float)i_nE;
    else
        mean_time_loop = mean_time_loop + (1/number_of_data)*((float)i_nE-mean_time_loop);
    return mean_time_loop;
}

void convert::flip()
{
    switch(orientation)
    {
        case 90:    addrxBack=addrx;
                    addrx=addry;
                    addry=height-(addrxBack+1);
                    break;
        case 180:   addrx=height-(addrx+1);
                    addry=width-(addry+1);
                    break;
        case 270:   addrxBack=addrx;
                    addrx=width-(addry+1);
                    addry=addrxBack;
                    break;
        default: break;
    }
}
