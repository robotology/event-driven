#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
//#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <string>

//#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/all.h>


class yarpDefault
{
    public:
        yarpDefault();
        yarpDefault(std::string, std::string, std::string, std::string);
        ~yarpDefault();
  
        yarp::os::BufferedPort<emorph::vBottle> inputPort;
        yarp::sig::ImageOf<yarp::sig::PixelRgb> imageToWrite;
        
        yarp::os::Bottle output;
        yarp::os::Port imagePort;    
        
        yarp::os::BufferedPort<yarp::os::Bottle> outputNeuronStatePort;
//        yarp::os::BufferedPort<yarp::os::Bottle> bufferPort;

};
