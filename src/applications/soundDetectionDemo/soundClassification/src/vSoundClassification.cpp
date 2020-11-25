//
// Created by Daniel Gutierrez-Galan
// dgutierrez@atc.us.es
// University of Seville
// 09/Nov/2020
//

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include "event-driven/vDraw.h"
#include <limits.h>

using namespace ev;
using namespace yarp::os;

class vSoundClassification : public RFModule, public Thread {

private:
    // Input port: type AE
    vReadPort< vector< AE> > input_port;
    // Output port: type AE
    vWritePort output_port;

    // Debug flag
    // If activated, the module shows info about the classification in the terminal
    bool is_debug_flag;

    // Dummy parameter
    int example_parameter;

    // Number to tones to be classified
    // ----------------------------------------------------------------------------
    // WARNING!! This number must be the same as the number of output neurons used
    // in SpiNNaker tones out population
    // ----------------------------------------------------------------------------
    int number_tones_output_neurons;

    // Tones' names
    // ----------------------------------------------------------------------------
    // WARNING!! The array size must be the same as the number of output neurons used
    // in SpiNNaker tones out population
    // ----------------------------------------------------------------------------
    const char *tones_output_neurons_names[6] = {"261 Hz", "349 Hz", "523 Hz", "698 Hz", "1046 Hz", "1396 Hz"};

    // Classification short term memory deque
    deque<int> pure_tones_short_term_memory;

    // Max. length of the deque for the short term memory
    int pure_tones_short_term_memory_size;

    // Visualizer output image
    cv::Mat image_tones_classification = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);

public:

    vSoundClassification() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        //set the module name used to name ports
        setName((rf.check("name", Value("/vSoundClassification")).asString()).c_str());

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
        yInfo() << "Flag is_debug_flat is: " << is_debug_flag;

        // Dummy parameter: 32 by default
        int default_example_parameter = 32;
        example_parameter = rf.check("example_parameter", 
                                    Value(default_example_parameter)).asInt();
        yInfo() << "Setting example_parameter parameter to: " << example_parameter;

         // Number of tones to classify: 6 by default
        int default_number_tones_output_neurons = 6;
        number_tones_output_neurons = rf.check("number_tones_output_neurons",
                                    Value(default_number_tones_output_neurons)).asInt();
        yInfo() << "Setting number_tones_output_neurons parameter to: " << number_tones_output_neurons;

        // Short term memory size: 100 by default
        int default_pure_tones_short_term_memory_size = 100;
        pure_tones_short_term_memory_size = rf.check("pure_tones_short_term_memory_size", 
                                    Value(default_pure_tones_short_term_memory_size)).asInt();
        yInfo() << "Setting pure_tones_short_term_memory_size parameter to: " << pure_tones_short_term_memory_size;

        // Do any other set-up required here

        // Start the asynchronous and synchronous threads
        yInfo() << "Starting the thread...";
        return Thread::start();
    }

    virtual double getPeriod()
    {
        // Period of synchrnous thread (in seconds)
        return 1.0;
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

    //synchronous thread
    virtual bool updateModule()
    {

        // Add any synchronous operations here, visualisation, debug out prints

        // Create variables
        int tones_histogram[number_tones_output_neurons];

        // Init the histogram
        for(int i = 0; i < number_tones_output_neurons; i++) {
            tones_histogram[i] = 0;
        }

        // Update the histogram with the information contained on the short term memory
        for(int i = 0; i < pure_tones_short_term_memory_size; i++){
            int deque_address = pure_tones_short_term_memory[i];
            tones_histogram[deque_address] = tones_histogram[deque_address] + 1;
        }

        // Look for both the max. and min. value, and their index within the array
        // ----------------------------------------------------------------------------
        // NOTE: The max. value is used to select the winner i.e. the neuron that fired the most
        // In addition, the max. and the min. value are used to normalize the histogram
        // ----------------------------------------------------------------------------
        int max_value = -1;
        int min_value = INT_MAX;

        int pos_max_value = -1;
        int pos_min_value = -1;

        for(int i = 0; i < number_tones_output_neurons; i++){
            int value = tones_histogram[i];

            if(value > max_value){
                max_value = value;
                pos_max_value = i;
            }
            if(value < min_value){
                min_value = value;
                pos_min_value = i;
            }
        }
        
        // Define the image settings

        // Horizontal and vertical margins: set to 3% of the image height
        float margin = image_tones_classification.cols * 0.03;

        // The image is divided by slices (one slice for each tone)
        // ----------------------------------------------------------------------------
        // NOTE: We took into account the horizontal margins (margin * 2)
        // ----------------------------------------------------------------------------
        float slot_size = (image_tones_classification.cols - (margin * 2)) / number_tones_output_neurons;
        
        // The circle's radius for each tone is defined as the 80% of the half of a slice
        // This way, we are sure there is some space between two consecutive circles
        float radius = (slot_size / 2.0) * 0.8;

        // ----------------------------------------------------------------------------
        //                          WINNER TEXT DRAW
        // First, plot the text indicating the winner neuron
        char winner_text[64];
        strcpy(winner_text, "Detected tone: ");
        strcat(winner_text, tones_output_neurons_names[pos_max_value]);

        cv::putText(image_tones_classification, winner_text, cv::Point(margin, margin * 2), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Vec3b(255, 255, 255), 0.5);
        // ----------------------------------------------------------------------------
        
        // Then, let's process each neuron
        for(int i = 0; i < number_tones_output_neurons; i++){

            // First, calculate the x position accordint its slice
            // It will be the neuron id * slot_size plus half of the slot size 
            // (in order to set the X in the center of the slice)
            float center_pos = margin + (i * slot_size) + (slot_size / 2.0);

            // By default, the circle is red
            cv::Vec3b c = cv::Vec3b(0, 0, 255);

            // If this neuron is the winner
            if(i == pos_max_value) {
                // Change color to green
                c = cv::Vec3b(0, 255, 0);
            }

            // ----------------------------------------------------------------------------
            //                          CIRCLE & CIRCLE'S TEXT DRAW
            // Draw the circle
            cv::circle(image_tones_classification, cv::Point(center_pos, 5 * (image_tones_classification.rows/6)), radius, c, cv::FILLED);
            // Add the tone name inside the circle
            cv::putText(image_tones_classification, tones_output_neurons_names[i], cv::Point(margin + (i * slot_size) + 5, 5 * (image_tones_classification.rows/6)),cv::FONT_HERSHEY_PLAIN, 0.5, cv::Vec3b(0, 0, 0), 0.5);
            // ----------------------------------------------------------------------------

            // ----------------------------------------------------------------------------
            //                          HISTOGRAM DRAW
            // First, get the normalized value of the neuron activity
            float normalized_histogram_value = (tones_histogram[i] - min_value) * 100.0 / (max_value - min_value);

            // Define left top point
            float rec_x_top_left = margin + (i * slot_size) + 10;
            // The y value is the bottom reference value minus the normalized value (the width of the histogram bar)
            float rec_y_top_left = 4 * (image_tones_classification.rows/6) - normalized_histogram_value;
            cv::Point p1(rec_x_top_left, rec_y_top_left);

            // Define right bottom point
            // X value is the same as left top point plus the diameter of the circle (radius * 2)
            float rec_x_bottom_right = rec_x_top_left + (radius * 2);
            float rec_y_bottom_right = 4 * (image_tones_classification.rows/6);
            cv::Point p2(rec_x_bottom_right, rec_y_bottom_right);

            // Create the rectangle
            cv::rectangle(image_tones_classification, p1, p2, c);
            // Add the normalized value to the rectangle
            cv::putText(image_tones_classification, std::to_string((int)normalized_histogram_value), cv::Point(margin + (i * slot_size) + 10, 4 * (image_tones_classification.rows/6)),cv::FONT_HERSHEY_PLAIN, 0.8, cv::Vec3b(255, 255, 255), 0.5);
            // ----------------------------------------------------------------------------
        }
        
        // If debug flag is enabled
        if(is_debug_flag == true){
            // Print the winner
            yInfo() << "Winner neuron is: " << pos_max_value;
        }

        // Show the image
        cv::imshow("tones_out_result", image_tones_classification);
        cv::waitKey(10);
         
        return Thread::isRunning();
    }

    // Asynchronous thread run forever
    void run()
    {
        // YARP timestamp
        Stamp yarpstamp;

        // Output queue: type AE (not used)
        deque<AE> out_queue;
        // Output event: type AE (not used)
        AE out_event;

        // Raw address received from the input port
        int address = 0;

        // Initialize the short term memory deque
        for(int i = 0; i < pure_tones_short_term_memory_size; i++) {
            pure_tones_short_term_memory.push_back(0);
        }

        // Forever...
        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            // Do asynchronous processing here
            for(auto &qi : *q) {

                // Get the raw AER address
                address = qi._coded_data;
                
                // If debug flag enabled, print it out
                if (is_debug_flag == true) {
                    yDebug() << "Event received and decoded address: " << address;
                }
                
                // If the received event is valid
                if ((address >= 0) && (address < number_tones_output_neurons)) {
                    // Add the new event into the buffer
                    pure_tones_short_term_memory.push_back(address);
                    // And remove the oldest element
                    pure_tones_short_term_memory.pop_front();
                } else {
                    // Otherwise
                    yWarning() << "Not recognized event detected...";
                    address = -1;
                }
            }

            // After processing the packet output the results
            // (only if there is something to output
            if(out_queue.size()) {
                // Write out the output events in the output port
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
    rf.setDefaultConfigFile( "vSoundClassification.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vSoundClassification instance;
    return instance.runModule(rf);
}
