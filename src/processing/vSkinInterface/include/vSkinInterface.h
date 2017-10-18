#ifndef __VSKININTERFACE__
#define __VSKININTERFACE__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <iCub/eventdriven/all.h>

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
// SKININTERFACE
/*////////////////////////////////////////////////////////////////////////////*/

class skinInterface : public yarp::os::Thread
{
private:

    //data structures and ports
    queueAllocator inputPort;
    yarp::os::Port outputPort;

    //variables
    
    //diagnostics
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;

public:

    skinInterface() {}

    bool open(std::string name);
    
    //bool threadInit();
    void onStop();
    void run();
    //void threadRelease();

};





#endif
