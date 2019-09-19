
#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

class exampleModule : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;

public:

    exampleModule() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        setName((rf.check("name", Value("/example-module")).asString()).c_str());
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }

        return Thread::start();
    }

    virtual double getPeriod()
    {
        return 1.0; //period of synchrnous thread
    }

    bool interruptModule()
    {
        return Thread::stop();
    }

    void onStop()
    {
        //close ports etc.
        input_port.close();
    }

    //synchronous thread
    virtual bool updateModule()
    {

        //add any synchronous operations here

        return Thread::isRunning();
    }

    //asynchronous thread
    void run()
    {
        Stamp yarpstamp;

        while(true) {

            unsigned int nqs = input_port.queryunprocessed();
            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            //do asynchronous processing here

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
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "sample_module.ini" );
    rf.configure( argc, argv );

    /* create the module */
    exampleModule instance;
    return instance.runModule(rf);
}
