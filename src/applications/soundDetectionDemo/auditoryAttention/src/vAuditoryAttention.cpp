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
    const char *sound_source_names[6] = {"90 L", "60 L", "30 L", "30 R", "60 R", "90 R"};

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
        /*int max = 0;
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

        }

        if(is_debug_flag == true){
            // Print the winer
            yInfo() << "Winer neuron is: " << pos_max;
        }

        cv::imshow("sound_source_result", image_sound_localization);
        cv::waitKey(10);*/
        int max = 0;
        int pos_max = 0;

        for(int i = 0; i < soundsource_short_term_memory_size; i++){
            if(soundsource_short_term_memory[i] > max){
                max = soundsource_short_term_memory[i];
                pos_max = i;
            }
        }

        // ----------------------------------------------------------------------------
        //                          CLEAR IMAGE
        // Remove all the previous plot
        image_sound_localization = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);
        
        float margin = image_sound_localization.cols * 0.03;
    
        float slot_angle = 180.0 / number_sound_source_neurons;
        
        float radius = (slot_angle / 2.0) * 0.8;

        float semicircle_radius = 150.0;

        float semicircle_x = image_sound_localization.cols / 2;
        float semicircle_y = image_sound_localization.rows - margin;

        cv::Point pref(semicircle_x, semicircle_y);

        float angle_betta = 0.0;
        float angle_alfa = 0.0;
        float angle_tetta = 0.0;
        
        float seg_ab = 0.0;
        float seg_bc = 0.0;
        float seg_ca = 0.0;

        // Draw reference horizontal line
        cv::Point p1(margin, image_sound_localization.rows - margin), p2(image_sound_localization.cols - margin, image_sound_localization.rows - margin);
        cv::line(image_sound_localization, p1, p2, cv::Scalar(255, 255, 255), 2);

        // Draw reference vertical line
        cv::Point p3(image_sound_localization.cols / 2, margin), p4(image_sound_localization.cols / 2, image_sound_localization.rows - margin);
        //cv::line(image_sound_localization, p3, p4, cv::Scalar(255, 255, 255), 2);

        // Draw text indicating the winner
        char winner_text[64];
        strcpy(winner_text, "Sound source placed at ");
        strcat(winner_text, sound_source_names[pos_max]);
        cv::putText(image_sound_localization, winner_text, cv::Point(margin , margin*2),cv::FONT_HERSHEY_PLAIN, 1.0, cv::Vec3b(255, 255, 255), 0.5);
        for(int i = 0; i < number_sound_source_neurons; i++){
            
            angle_tetta = 90.0;
            angle_betta = i * slot_angle;
            angle_alfa = 180.0 - angle_tetta - angle_betta;

            seg_ab = semicircle_radius;
            float angle_betta_rads = angle_betta * (M_PI / 180.0);
            seg_bc = seg_ab * cos(angle_betta_rads);
            seg_ca = seg_ab * sin(angle_betta_rads);

            cv::Point pdest(semicircle_x - seg_bc, semicircle_y - seg_ca);
            cv::line(image_sound_localization, pref, pdest, cv::Scalar(255, 255, 255), 2);

            float angle_betta_bisector = angle_betta + (slot_angle / 2.0);

            float angle_betta_bisector_rads = angle_betta_bisector * (M_PI / 180.0);
            float x = seg_ab * cos(angle_betta_bisector_rads);
            float y = seg_ab * sin(angle_betta_bisector_rads);

            // By default, the circle is red
            cv::Vec3b c = cv::Vec3b(0, 0, 255);
            if(i == pos_max) {
                // Is the winner --> draw with green
                c = cv::Vec3b(0, 255, 0);
            }
            if(i < number_sound_source_neurons){
                cv::circle(image_sound_localization, cv::Point(semicircle_x-x, semicircle_y-y), radius, c, cv::FILLED);
            }
        }

        if(is_debug_flag == true){
            // Print the winer
            yInfo() << "Winer neuron is: " << pos_max;
        }

        cv::imshow("sound_source_result", image_sound_localization);
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
