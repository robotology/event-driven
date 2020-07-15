
#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <iomanip>
#include <sstream>

using namespace ev;
using namespace yarp::os;
using std::vector;

class hexViewer : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;
    unsigned int mask;
    unsigned int bits_to_check;
    int cols;

public:

    hexViewer() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/vHexviewer")).asString()).c_str());

        //open io ports
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }

        cols = rf.check("cols", Value(4)).asInt();
        std::stringstream ss; ss.str("");

        bits_to_check = 0; mask = 0;
        if(rf.check("mask")) {
            std::string maskstring = rf.check("mask", Value("x")).asString();
            if(maskstring.empty()) {
                ss.str(""); ss << rf.find("mask").asInt();
                maskstring = ss.str();
            }

            //navigate through
            int bit = 0;
            for(std::string::const_reverse_iterator c = maskstring.rbegin();
                c != maskstring.rend(); c++, bit++) {

                if(*c == '1') {
                    mask |= (1 << bit);
                    bits_to_check |= (1 << bit);
                } else if(*c == '0') {
                    bits_to_check |= (1 << bit);
                }

            }

            ss.str("");
            if(bits_to_check) {
                for(int i = 31; i >= 0; i--) {
                    if(bits_to_check & (1 << i)) {
                        if(mask & (1 << i))
                            ss << "1";
                        else
                            ss << "0";
                    } else {
                        ss << "x";
                    }
                }
                ss << "b";
                yInfo() << "Showing only events with bits: " << ss.str();
            } else {
                yInfo() << "No valid mask provided - showing all events";
            }

        } else {
            yInfo() << "No mask provided - showing all events";
        }

        //start the asynchronous and synchronous threads
        return Thread::start();
    }

    virtual double getPeriod()
    {
        return 1.0; //period of synchrnous thread
    }

    bool interruptModule()
    {
        //if the module is asked to stop ask the asynchrnous thread to stop
        return Thread::stop();
    }

    void onStop()
    {
        //when the asynchrnous thread is asked to stop, close ports and do
        //other clean up
        input_port.close();
    }

    //synchronous thread
    virtual bool updateModule()
    {
        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        Stamp yarpstamp;
        std::cout << std::hex << std::setfill('0') << std::internal << std::uppercase;
        int coli = 0;

        while(!Thread::isStopping()) {

            int qs = input_port.queryunprocessed() - 1;
            if(qs < 1) qs = 1;

            for(int i = 0; i < qs; i++)
            {
                const vector<AE> * q = input_port.read(yarpstamp);
                if(!q) return;
                for(auto &v : (*q))
                {
                    if((v._coded_data & bits_to_check) == mask) {
                        if(coli++ % cols == 0) std::cout << std::endl;
                        std::cout << "0x" << std::setw(8) << v._coded_data << " ";
                    }
                }
            }

            if(coli) {
                coli = 0;
                std::cout << std::endl << "==";
                std::cout.flush();
            }
        }
    }
};

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "vHexviewer.ini" );
    rf.configure( argc, argv );

    /* create the module */
    hexViewer instance;
    return instance.runModule(rf);
}
