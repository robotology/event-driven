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
    unsigned int lossCount;
    unsigned int bytesperevent;
    bool msgflag;

    std::vector<unsigned char> *readBuffer;
    std::vector<unsigned char> *accessBuffer;
    std::vector<unsigned char> buffer1;
    std::vector<unsigned char> buffer2;
    std::vector<unsigned char> discardbuffer;

    yarp::os::Semaphore safety;
    yarp::os::Semaphore signal;
    bool bufferedreadwaiting;

public:

    vDevReadBuffer();

    bool initialise(std::string devicename, unsigned int bufferSize = 0,
                    unsigned int readSize = 0);

    //virtual bool threadInit();      //run before thread starts
    virtual void run();             //main function
    //virtual void onStop();          //run when stop() is called (first)
    virtual void threadRelease();   //run after thread stops (second)
    std::vector<unsigned char>& getBuffer(unsigned int &nBytesRead, unsigned int &nBytesLost);

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
    int countLoss;
    double rate;
    int prevAEs;
    double prevTS;
    yarp::os::Stamp vStamp;

    //data buffer thread
    vDevReadBuffer deviceReader;


public:

    device2yarp();
    bool initialise(std::string moduleName = "", bool strict = false,
                    std::string deviceName = "", unsigned int bufferSize = 800000,
                    unsigned int readSize = 1024);
    virtual void run();
    virtual void threadRelease();
    virtual void afterStart(bool success);


};




#endif
