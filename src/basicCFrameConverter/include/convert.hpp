#ifndef C_CONVERTER
#define C_CONVERTER

//std
#include <iostream>
#include <ctime>
#include <list>

//#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>

#include "iCub/emorph/eventUnmask.h"
#include "iCub/emorph/eventUnmaskDVS128.h"
#include "iCub/emorph/eventUnmaskICUB.h"
#include "iCub/emorph/eventUnmaskICUBcircBuf.h"


//#define _DEBUG
#define THRATE 5
#define CONTRAST 20

//class convert:public yarp::os::RateThread
class convert:public yarp::os::Thread
{
public:
	convert(unsigned int, unsigned int, unsigned int, std::string, unsigned int, unsigned int);
	~convert();
    void setBuffer(char*, unsigned int);
    
    void initFrames();
    void createFrames();
	void sendFrames(yarp::sig::ImageOf<yarp::sig::PixelMono16>&, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> >&);

    void run();
private:
    int sign(int);
    float mean_event(int);
    void flip();

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > portLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > portRight;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> base_img;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> imgLeft;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> imgRight;

    yarp::os::Semaphore mutex;

    int height;
    int width;
    unsigned int orientation;

    emorph::eunmask::eventUnmask *unmasker;

    unsigned int addrx, addrxBack;
    unsigned int addry;
    unsigned int eye;
    int polarity;
    unsigned int timestamp;
    unsigned int refts;
    unsigned int accTime;
    bool startNewFrame; 

    time_t stime;
    time_t ctime;
};
#endif //C_CONVERTER
