//
// Created by Daniel Gutierrez-Galan
// dgutierrez@atc.us.es
// University of Seville
// 10/Nov/2020
//

#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

class vAuditoryAttention : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;
    vWritePort output_port;

    bool is_debug_flag;
    double example_parameter;

    int number_sound_source_neurons;

    int soundsource_short_term_memory_size;
    int soundsource_short_term_memory[soundsource_short_term_memory_size];


public:

    vAuditoryAttention() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        //set the module name used to name ports
        setName((rf.check("name", Value("/vAuditoryAttention")).asString()).c_str());

        //open io ports
        yInfo() << "Opening input port...";
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }
        yInfo() << "Opening output port...";
        output_port.setWriteType(AE::tag);
        if(!output_port.open(getName() + "/AE:o")) {
            yError() << "Could not open output port";
            return false;
        }

        //read flags and parameters
        is_debug_flag = rf.check("is_debug_flag") &&
                rf.check("is_debug_flag", Value(true)).asBool();
        yInfo() << "Flag is_debug_flag is: " << is_debug_flag;

        double default_value = 0.1;
        example_parameter = rf.check("example_parameter",
                                     Value(default_value)).asDouble();
        yInfo() << "Setting example_parameter parameter to: " << example_parameter;

        int default_number_sound_source_neurons = 6;
        number_sound_source_neurons = rf.check("number_sound_source_neurons",
                                    Value(default_number_sound_source_neurons)).asInt();
        yInfot() << "Setting number_sound_source_neurons parameter to: " << number_sound_source_neurons;

        int default_soundsource_short_term_memory_size = 6;
        soundsource_short_term_memory_size = rf.check("soundsource_short_term_memory_size",
                                    Value(default_number_sound_source_neurons)).asInt();
        yInfot() << "Setting soundsource_short_term_memory_size parameter to: " << soundsource_short_term_memory_size;

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
        yInfo() << "Closing output port...";
        output_port.close();
        yInfo() << "Module has been closed!";
    }

    //synchronous thread
    virtual bool updateModule()
    {

        //add any synchronous operations here, visualisation, debug out prints

        // Try to print here the result of the pure tones classification and the sound source localization


        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        Stamp yarpstamp;
        deque<AE> out_queue;
        AE out_event;
        int address = 0;

        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            // Initialize the buffer
            for(int i = 0; i < soundsource_short_term_memory_size; i++){
                soundsource_short_term_memory[i] = 0;
            }

            //do asynchronous processing here
            for(auto &qi : *q) {

                //here you could try modifying the data of the event before
                //pushing to the output q

                // First, check from which SpiNNaker sub-network is the event coming from

                // If it is an event from pure tones classification
                // then count and check the winer


                // If it is an event from sound source localization
                // then count and check the winer

                // Get the real AER address
                address = qi._coded_data;

                if (is_debug_flag == true) {
                    yDebug() << "Event received and decoded address: " << address;
                }

                // If the received event is valid
                if ((address >= 0) && (address < number_sound_source_neurons)) {
                    // Add the new event into the buffer
                    soundsource_short_term_memory[address] = soundsource_short_term_memory[address] + 1;
                } else {
                    // Otherwise
                    yWarning() << "Not recognized event detected...";
                    address = -1;
                }
 
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
    rf.setDefaultConfigFile( "vAuditoryAttention.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vAuditoryAttention instance;
    return instance.runModule(rf);
}
