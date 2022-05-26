
#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

class exampleModule : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;
    vWritePort output_port;

    bool example_flag;
    double example_parameter;

public:

    exampleModule() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/example-module")).asString()).c_str());

        //open io ports
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }
        output_port.setWriteType(AE::tag);
        if(!output_port.open(getName() + "/AE:o")) {
            yError() << "Could not open input port";
            return false;
        }

        //read flags and parameters
        example_flag = rf.check("example_flag") &&
                rf.check("example_flag", Value(true)).asBool();
        double default_value = 0.1;
        example_parameter = rf.check("example_parameter",
                                     Value(default_value)).asFloat64();

        //do any other set-up required here

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

        //add any synchronous operations here, visualisation, debug out prints


        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        Stamp yarpstamp;
        deque<AE> out_queue;

        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            //do asynchronous processing here
            for(auto &qi : *q) {

                //here you could try modifying the data of the event before
                //pushing to the output q

                //the position of the event (qi.x, qi.y)
                //the polarity of the event (qi.polarity)
                //the timstamp (qi.stamp)
                //or remove some events based on a condition?
                //if(qi.x < 100) only takes the left 1/3 of events

                out_queue.push_back(qi);   
            }

            //after processing the packet output the results
            //(only if there is something to output
            if(out_queue.size()) {
                output_port.write(out_queue, yarpstamp);
                out_queue.clear();
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
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "sample_module.ini" );
    rf.configure( argc, argv );

    /* create the module */
    exampleModule instance;
    return instance.runModule(rf);
}
