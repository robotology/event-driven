//
// Created by Daniel Gutierrez-Galan
// dgutierrez@atc.us.es
// University of Seville
// 19/Nov/2020
//

#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

class vSpinnakerEventsMapper : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;
    vWritePort output_port_tones;
    vWritePort output_port_soundsource;

    bool is_debug_flag;
    int example_parameter;

public:

    vSpinnakerEventsMapper() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        //set the module name used to name ports
        setName((rf.check("name", Value("/vSpinnakerEventsMapper")).asString()).c_str());

        //open io ports
        yInfo() << "Opening input port...";
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }
        yInfo() << "Opening output tones port...";
        output_port_tones.setWriteType(AE::tag);
        if(!output_port_tones.open(getName() + "/tones:o")) {
            yError() << "Could not open output port";
            return false;
        }
        yInfo() << "Opening output soundsource port...";
        output_port_soundsource.setWriteType(AE::tag);
        if(!output_port_soundsource.open(getName() + "/soundsource:o")) {
            yError() << "Could not open output port";
            return false;
        }

        //read flags and parameters
        is_debug_flag = rf.check("is_debug_flag") &&
                rf.check("is_debug_flag", Value(true)).asBool();
        yInfo() << "Flag is_debug_flat is: " << is_debug_flag;

        int default_example_parameter = 32;
        example_parameter = rf.check("example_parameter", 
                                    Value(default_example_parameter)).asInt();
        yInfo() << "Setting example_parameter parameter to: " << example_parameter;

        //do any other set-up required here

        //start the asynchronous and synchronous threads
        yInfo() << "Starting the thread...";
        return Thread::start();
    }

    virtual double getPeriod()
    {
        return 1.0; //period of synchrnous thread
    }

    bool interruptModule()
    {
        //if the module is asked to stop ask the asynchrnous thread to stop
        yInfo() << "Interrupting the module: stopping thread...";
        return Thread::stop();
    }

    void onStop()
    {
        //when the asynchrnous thread is asked to stop, close ports and do
        //other clean up
        yInfo() << "Stopping the module...";
        yInfo() << "Closing input port...";
        input_port.close();
        yInfo() << "Closing output tones port...";
        output_port_tones.close();
        yInfo() << "Closing output soundsource port...";
        output_port_soundsource.close();
        yInfo() << "Module has been closed!";
    }

    //synchronous thread
    virtual bool updateModule()
    {

        //add any synchronous operations here, visualisation, debug out prints

        // Try to print here the spikegram, the histogram, the mean activity, the heatmap...


        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        Stamp yarpstamp;
        deque<AE> out_queue_tones;
        deque<AE> out_queue_soundsource;
        AE out_event;
        int address = 0;

        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            //do asynchronous processing here
            for(auto &qi : *q) {

                //here you could try modifying the data of the event before
                //pushing to the output q

                // Get the real AER address

                if (is_debug_flag == true) {
                    yDebug() << "Event received and decoded address: " << address;
                }
                
            }

            //after processing the packet output the results
            //(only if there is something to output
            if(out_queue_tones.size()) {
                output_port_tones.write(out_queue_tones, yarpstamp);
                out_queue_tones.clear();
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
    rf.setDefaultConfigFile( "vSpinnakerEventsMapper.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vSpinnakerEventsMapper instance;
    return instance.runModule(rf);
}
