//
// Created by Daniel Gutierrez-Galan
// dgutierrez@atc.us.es
// University of Seville
// 09/Nov/2020
//

#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

class vSoundClassification : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;
    vWritePort output_port;

    bool is_debug_flag;
    int example_parameter;

    int number_tones_output_neurons;

    deque<int> pure_tones_short_term_memory;
    int pure_tones_short_term_memory_size;

public:

    vSoundClassification() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        //set the module name used to name ports
        setName((rf.check("name", Value("/vSoundClassification")).asString()).c_str());

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
        yInfo() << "Flag is_debug_flat is: " << is_debug_flag;

        int default_example_parameter = 32;
        example_parameter = rf.check("example_parameter", 
                                    Value(default_example_parameter)).asInt();
        yInfo() << "Setting example_parameter parameter to: " << example_parameter;

        int default_number_tones_output_neurons = 6;
        number_tones_output_neurons = rf.check("number_tones_output_neurons",
                                    Value(default_number_tones_output_neurons)).asInt();
        yInfo() << "Setting number_tones_output_neurons parameter to: " << number_tones_output_neurons;

        int default_pure_tones_short_term_memory_size = 100;
        pure_tones_short_term_memory_size = rf.check("pure_tones_short_term_memory_size", 
                                    Value(default_pure_tones_short_term_memory_size)).asInt();
        yInfo() << "Setting pure_tones_short_term_memory_size parameter to: " << pure_tones_short_term_memory_size;

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

        // Try to print here the spikegram, the histogram, the mean activity, the heatmap...

        // Create variables
        int tones_histogram[number_tones_output_neurons];

        // Init the histogram
        for(int i = 0; i < number_tones_output_neurons; i++) {
            tones_histogram[i] = 0;
        }

        // Count the elements
        for(int i = 0; i < pure_tones_short_term_memory_size; i++){
            int deque_address = pure_tones_short_term_memory[i];
            tones_histogram[deque_address] = tones_histogram[deque_address] + 1;
        }

        // Look for the max value
        int max_value = -1;
        int pos_max_value = -1;

        for(int i = 0; i < number_tones_output_neurons; i++){
            int value = tones_histogram[i];
            if(value > max_value){
                max_value = value;
                pos_max_value = i;
            }
        }

        // Print the winer
        yInfo() << "Winer neuron is: " << pos_max_value; 

        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        Stamp yarpstamp;
        deque<AE> out_queue;
        AE out_event;
        int address = 0;

        // Initialize the queue
        for(int i = 0; i < pure_tones_short_term_memory_size; i++) {
            pure_tones_short_term_memory.push_back(0);
        }

        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            //do asynchronous processing here
            for(auto &qi : *q) {

                //here you could try modifying the data of the event before
                //pushing to the output q

                // Get the real AER address
                address = qi._coded_data;
                
                if (is_debug_flag == true) {
                    yDebug() << "Event received and decoded address: " << address;
                }
                
                // If the received event is valid
                if ((address >= 0) && (address < number_tones_output_neurons)) {
                    // Add the new event into the buffer
                    pure_tones_short_term_memory.push_back(address);
                    pure_tones_short_term_memory.pop_front();
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
    rf.setDefaultConfigFile( "vSoundClassification.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vSoundClassification instance;
    return instance.runModule(rf);
}
