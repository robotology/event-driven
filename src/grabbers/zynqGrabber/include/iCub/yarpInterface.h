#ifndef __EVENTDRIVENYARPINTERFACE__
#define __EVENTDRIVENYARPINTERFACE__

#define THRATE 1

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <string>

/******************************************************************************/
//vDevReadBuffer
/******************************************************************************/
class vDevReadBuffer : public yarp::os::Thread {

private:

    //parameters
    unsigned int bufferSize;
    unsigned int readSize;

    //internal variables/storage
    int fd;
    unsigned int readCount;
    unsigned int bytesperevent;
    unsigned int bytestoread;
    bool msgflag;

    std::vector<int32_t> *readBuffer;
    std::vector<int32_t> *accessBuffer;
    std::vector<int32_t> buffer1;
    std::vector<int32_t> buffer2;
    std::vector<int32_t> discardbuffer;

    yarp::os::Semaphore safety;
    yarp::os::Semaphore signal;
    bool bufferedreadwaiting;

public:

    vDevReadBuffer();

    bool initialise(std::string devicename, unsigned int bufferSize = 0,
                    unsigned int readSize = 0, unsigned int bytesperevent = 8);

    //virtual bool threadInit();      //run before thread starts
    virtual void run();             //main function
    //virtual void onStop();          //run when stop() is called (first)
    virtual void threadRelease();   //run after thread stops (second)
    unsigned int getBuffer(std::vector<int32_t> *bufferpointer);

};

/******************************************************************************/
//device2yarp
/******************************************************************************/
class device2yarp : public yarp::os::RateThread {

private:

    //parameters
    bool strict;

    //internal variables
    yarp::os::BufferedPort<emorph::vBottleMimic> portvBottle;
    int countAEs;
    double rate;
    int prevAEs;
    double prevTS;
    yarp::os::Stamp vStamp;

    //data buffer thread
    vDevReadBuffer deviceReader;


public:

    device2yarp();
    bool initialise(std::string moduleName = "", bool strict = false,
                    std::string deviceName = "", unsigned int bufferSize = 5000,
                    unsigned int readSize = 128);
    virtual void run();
    virtual void threadRelease();
    virtual void afterStart();


};




#endif
