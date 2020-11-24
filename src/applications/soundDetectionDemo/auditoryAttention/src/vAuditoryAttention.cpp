//
// Created by Daniel Gutierrez-Galan
// dgutierrez@atc.us.es
// University of Seville
// 10/Nov/2020
//

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include "event-driven/vDraw.h"
using namespace ev;
using namespace yarp::os;

class vAuditoryAttention : public RFModule, public Thread {

private:

    vReadPort< vector<AE> > input_port;
    vWritePort output_port;

    bool is_debug_flag;
    double example_parameter;

    int number_sound_source_neurons;
    const char *sound_source_names[6] = {"90º L", "60º L", "30º L", "30º R", "60º R", "90º R"};

    int soundsource_short_term_memory_size;
    int soundsource_short_term_memory[512];

    cv::Mat image_sound_localization = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);


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
        yInfo() << "Setting number_sound_source_neurons parameter to: " << number_sound_source_neurons;

        int default_soundsource_short_term_memory_size = 6;
        soundsource_short_term_memory_size = rf.check("soundsource_short_term_memory_size",
                                    Value(default_number_sound_source_neurons)).asInt();
        yInfo() << "Setting soundsource_short_term_memory_size parameter to: " << soundsource_short_term_memory_size;

        //do any other set-up required here
        for(int i = 0; i < soundsource_short_term_memory_size; i++){
            soundsource_short_term_memory[i] = 0;
        }

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
        int max = 0;
        int pos_max = 0;

        for(int i = 0; i < soundsource_short_term_memory_size; i++){
            if(soundsource_short_term_memory[i] > max){
                max = soundsource_short_term_memory[i];
                pos_max = i;
            }
        }
        
        float margin = image_sound_localization.cols * 0.03;
    
        float slot_size = (image_sound_localization.cols - (margin * 2)) / number_sound_source_neurons;
        
        float radius = (slot_size / 2.0) * 0.8;
        
        for(int i = 0; i < number_sound_source_neurons; i++){
            float center_pos = margin + (i * slot_size) + (slot_size / 2.0);

            // By default, the circle is red
            cv::Vec3b c = cv::Vec3b(0, 0, 255);

            if(i == pos_max) {
                // Is the winner --> draw with green
                c = cv::Vec3b(0, 255, 0);
            }
            cv::circle(image_sound_localization, cv::Point(center_pos, image_sound_localization.rows/2), radius, c, cv::FILLED);
            cv::putText(image_sound_localization, sound_source_names[i], cv::Point(margin + (i * slot_size) + 5, image_sound_localization.rows/2),cv::FONT_HERSHEY_PLAIN, 0.5, cv::Vec3b(0, 0, 0), 0.5);
        }

        if(is_debug_flag == true){
            // Print the winer
            yInfo() << "Winer neuron is: " << pos_max;
        }

        cv::imshow("tones_out_result", image_sound_localization);
        cv::waitKey(10);

        for(int i = 0; i < soundsource_short_term_memory_size; i++){
            soundsource_short_term_memory[i] = 0;
        }

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
                //soundsource_short_term_memory[i] = 0;
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
