//
// Created by Daniel Gutierrez-Galan
// dgutierrez@atc.us.es
// University of Seville
// 10/Nov/2020
//

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include "event-driven/vDraw.h"
#include <limits.h>

using namespace ev;
using namespace yarp::os;

class vAuditoryAttention : public RFModule, public Thread {

private:

    // Input port: type AE
    vReadPort< vector<AE> > input_port;
    
    // Output port: type AE
    vWritePort output_port;

    // Debug flag
    // If activated, the module shows info about the classification in the terminal
    bool is_debug_flag;

    // Number of sound source positions
    int number_sound_source_neurons;
    // ----------------------------------------------------------------------------
    // WARNING!! This number must be the same as the number of output neurons used
    // in SpiNNaker sound sources population
    // ----------------------------------------------------------------------------

    // Temporal histogram of the sound source neurons
    // ----------------------------------------------------------------------------
    // WARNING!! This number must be the same as the number of output neurons used
    // in SpiNNaker sound sources population
    // ----------------------------------------------------------------------------
    int soundsource_short_term_memory[512];

    // Period
    double update_period;

    // TS
    unsigned int ts;

    // YARP timestamp
    Stamp yarpstamp;

    // Output queue: type AE
    deque<AE> out_queue;

    // Output event: type AE
    AE out_event;

    // Visualizer output image
    int image_sound_localization_width = 320;
    int image_sound_localization_height = 240;
    cv::Mat image_sound_localization = cv::Mat::zeros(cv::Size(image_sound_localization_width, image_sound_localization_height), CV_8UC3);

public:

    vAuditoryAttention() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        // Set the module name used to name ports
        setName((rf.check("name", Value("/vAuditoryAttention")).asString()).c_str());

        // Open input port
        yInfo() << "Opening input port...";
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }
        // Open output port
        yInfo() << "Opening output port...";
        output_port.setWriteType(AE::tag);
        if(!output_port.open(getName() + "/AE:o")) {
            yError() << "Could not open output port";
            return false;
        }

        // Read flags and parameters

        // Debug flag: false by default
        is_debug_flag = rf.check("is_debug_flag") &&
                rf.check("is_debug_flag", Value(true)).asBool();
        yInfo() << "Flag is_debug_flag is: " << is_debug_flag;

        // Number of sound source positions to classify: 9 by default
        int default_number_sound_source_neurons = 9;
        number_sound_source_neurons = rf.check("number_sound_source_neurons",
                                    Value(default_number_sound_source_neurons)).asInt();
        yInfo() << "Setting number_sound_source_neurons parameter to: " << number_sound_source_neurons;

        // Period value: 1 sec by default
        double default_update_period = 1.0;
        update_period = rf.check("update_period",
                                    Value(default_update_period)).asDouble();
        yInfo() << "Setting update_period parameter to: " << update_period;

        // Do any other set-up required here

        // Set the histogram to zero
        for(int i = 0; i < number_sound_source_neurons; i++){
            soundsource_short_term_memory[i] = 0;
        }

        // Start the asynchronous and synchronous threads
        yInfo() << "Starting the thread...";
        return Thread::start();
    }

    virtual double getPeriod()
    {
        // Period of synchrnous thread (in seconds)
        return update_period;
    }

    bool interruptModule()
    {
        // If the module is asked to stop ask the asynchrnous thread to stop
        yInfo() << "Interrupting the module: stopping thread...";
        return Thread::stop();
    }

    void onStop()
    {
        // When the asynchrnous thread is asked to stop, close ports and do
        // other clean up
        yInfo() << "Stopping the module...";
        yInfo() << "Closing input port...";
        input_port.close();
        yInfo() << "Closing output port...";
        output_port.close();
        yInfo() << "Module has been closed!";
    }

    // Synchronous thread
    virtual bool updateModule()
    {

        // Find the maximum value inside the short term memory
        int max = 0;
        int pos_max = 0;

        for(int i = 0; i < number_sound_source_neurons; i++){
            if(soundsource_short_term_memory[i] > max){
                max = soundsource_short_term_memory[i];
                pos_max = i;
            }
        }

        // ----------------------------------------------------------------------------
        //                          CLEAR IMAGE
        // Remove all the previous plot
        image_sound_localization = cv::Mat::zeros(cv::Size(image_sound_localization_width, image_sound_localization_height), CV_8UC3);

        // Define the image settings

        // Horizontal and vertical margins: set to 3% of the image height
        float margin = image_sound_localization.cols * 0.03;

        // The image is divided by angles (one angle for each localization)
        // ----------------------------------------------------------------------------
        // NOTE: We took into account the horizontal line as reference: 180 degrees
        // ----------------------------------------------------------------------------
        float slot_angle = 180.0 / number_sound_source_neurons;
        
        // The circle's radius for each position is defined as the 80% of the half of a slice
        // This way, we are sure there is some space between two consecutive circles
        float radius = (slot_angle / 2.0) * 0.8;

        // ----------------------------------------------------------------------------
        //                          REFERENCE SEMICIRCLE DRAW
        // Radius of the semicircle
        float semicircle_radius = 150.0;

        // The center of the semicircle will be placed at the center of the image (x-axis)
        // at the bottom
        float semicircle_x = image_sound_localization.cols / 2;
        float semicircle_y = image_sound_localization.rows - margin;

        // Draw a small circle to mark the center of the semicircle. This will be the
        // reference point
        cv::Point pref(semicircle_x, semicircle_y);

        // Draw reference horizontal line
        cv::Point p1(margin, image_sound_localization.rows - margin), p2(image_sound_localization.cols - margin, image_sound_localization.rows - margin);
        cv::line(image_sound_localization, p1, p2, cv::Scalar(255, 255, 255), 2);

        // Draw reference vertical line
        cv::Point p3(image_sound_localization.cols / 2, margin), p4(image_sound_localization.cols / 2, image_sound_localization.rows - margin);

        // ----------------------------------------------------------------------------
        //                          WINNER TEXT DRAW
        // Draw text indicating the winner
        string winner_text;
        winner_text.append("Sound source placed in range ");
        cv::putText(image_sound_localization, winner_text, cv::Point(margin , margin*2),cv::FONT_HERSHEY_PLAIN, 1.0, cv::Vec3b(255, 255, 255), 0.5);

        string winner_range_text;
        winner_range_text.append("[");
        float range_start = (pos_max * slot_angle) - 90.0;
        string range_start_label(std::to_string(range_start));
        winner_range_text.append(range_start_label);
        winner_range_text.append(", ");
        float range_end = ((pos_max + 1) * slot_angle) -90.0;
        string range_end_label(std::to_string(range_end));
        winner_range_text.append(range_end_label);
        winner_range_text.append("]");

        cv::putText(image_sound_localization, winner_range_text, cv::Point(margin , margin*4),cv::FONT_HERSHEY_PLAIN, 1.0, cv::Vec3b(255, 255, 255), 0.5);
        
        // ----------------------------------------------------------------------------
        //                          LOCALIZATIONS DRAW
        float angle_betta = 0.0;
        float angle_alfa = 0.0;
        float angle_tetta = 0.0;
        
        float seg_ab = 0.0;
        float seg_bc = 0.0;
        float seg_ca = 0.0;
        
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

        // Copy the neurond ID
        out_event._coded_data = pos_max;
        // Copy the timestamp --> Is it necessary to pre-process?!!!!!!
        out_event.stamp = ts;
        // Copy the output event into the output queue
        out_queue.push_back(out_event); 

        for(int i = 0; i < number_sound_source_neurons; i++){
            soundsource_short_term_memory[i] = 0;
        }

        return Thread::isRunning();
    }

    // Asynchronous thread run forever
    void run()
    {
        // Raw address received from the input port
        int address = 0;

        // Forever...
        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            // Do asynchronous processing here
            for(auto &qi : *q) {

                // Get the real AER address
                address = qi._coded_data;
                ts = qi.stamp;

                // If debug flag enabled, print it out
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

            // After processing the packet output the results
            // (only if there is something to output
            if(out_queue.size()) {
                output_port.write(out_queue, yarpstamp);
                out_queue.clear();
            }
        }
    }
};

int main(int argc, char * argv[])
{
    // Initialize yarp network
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    // Prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vAuditoryAttention.ini" );
    rf.configure( argc, argv );

    // Create the module
    vAuditoryAttention instance;
    return instance.runModule(rf);
}
