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

class vCochleaEventsMapper : public RFModule, public Thread {

private:
    // Input port: type CochleaEvent
    vReadPort< vector< CochleaEvent > > input_port;
    // Output port: type AE
    vWritePort output_port;

    // Debug flag
    // If activated, the module shows info about the classification in the terminal
    bool is_debug_flag;

    // Dummy parameter
    int example_parameter;

    // Matrix of coincidence counters from MSO
    // ----------------------------------------------------------------------------
    // WARNING!! This matrix should have the same dimensions as the MSO model
    // used in the VHDL model
    // ----------------------------------------------------------------------------
    int mso_histogram[4][16];
    int mso_start_freq_channel = 13;

    // Visualizer output image
    cv::Mat image_mso_heatmap = cv::Mat::zeros(cv::Size(320,240), CV_8UC3);

public:

    vCochleaEventsMapper() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        // Set the module name used to name ports
        setName((rf.check("name", Value("/vCochleaEventsMapper")).asString()).c_str());

        // Open input port
        yInfo() << "Opening input port...";
        if(!input_port.open(getName() + "/CochleaEvent:i")) {
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
        
        // Debug flag: flase by default
        is_debug_flag = rf.check("is_debug_flag") &&
                rf.check("is_debug_flag", Value(true)).asBool();
        yInfo() << "Flag is_debug_flat is: " << is_debug_flag;

        // Dummy parameter: 32 by default
        int default_example_parameter = 32;
        example_parameter = rf.check("example_parameter", 
                                    Value(default_example_parameter)).asInt();
        yInfo() << "Setting example_parameter parameter to: " << example_parameter;

        // Initialize the mso coincidence counters to zero
        // ----------------------------------------------------------------------------
        // WARNING!! This values should match with the ones used before
        // ----------------------------------------------------------------------------
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 16; j++) {
                mso_histogram[i][j] = 0;
            }
        }

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

    // Synchronous thread
    virtual bool updateModule()
    {
        // Add any synchronous operations here, visualisation, debug out prints

        // Do any other set-up required here

        // Calculate max and min val (for normalization) and the position of the 
        // max value for using another color
        // ----------------------------------------------------------------------------
        // NOTE: The normalization is used also for generate the color code for the heatmap
        // ----------------------------------------------------------------------------
        float max = 0.0f;
        float min = FLT_MAX;
        int max_position_i = 0;
        int max_position_j = 0;

        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 16; j++) {
                float val = (float)(mso_histogram[i][j]);
                if(val > max){
                    max = val;
                    max_position_i = i;
                    max_position_j = j;
                }
                if(val < min){
                    min = val;
                }
            }
        }

        // ----------------------------------------------------------------------------
        //                          COLOR GRADIENT ESTIMATION
        // Calculate the color based on the normalized value
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 16; j++) {
                
                float pixelValue = (float)(mso_histogram[i][j]);
                float value = ((pixelValue - min) / (max - min));

                int aR = 0;   int aG = 0; int aB=255;     // RGB for our 1st color (blue in this case).
                int bR = 255; int bG = 0; int bB=0;       // RGB for our 2nd color (red in this case).

                int r = (float)(bR - aR) * value + aR;    // Evaluated as -255*value + 255.
                int g = (float)(bG - aG) * value + aG;    // Evaluates as 0.
                int b = (float)(bB - aB) * value + aB;    // Evaluates as 255*value + 0.

                // If the current rectangle is the winner, use the color green
                if((i == max_position_i) && (j == max_position_j)){
                    r = 0;
                    g = 255;
                    b = 0;
                }
                // ----------------------------------------------------------------------------
                //                          RECTANGLE DRAW
                // Draw the rectangle
                cv::Rect rect(j*20, i*60, 20, 60);
                cv::rectangle(image_mso_heatmap, rect, cv::Vec3b(b, g, r), -1);
            }
        }

        // Show the image
        cv::imshow("mso_heatmap", image_mso_heatmap);
        cv::waitKey(10);

        // Clean the mso matrix values before the next update
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 16; j++) {
                mso_histogram[i][j] = 0;
            }
        }

        return Thread::isRunning();
    }

    // Asynchronous thread run forever
    void run()
    {
        // YARP timestamp
        Stamp yarpstamp;
        
        // Output queue: type AE
        deque<AE> out_queue;

        // Output event: type AE
        AE out_event;

        // Raw address received from the input port
        int address = 0;

        // Initialize the first time the mso matrix values
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 16; j++) {
                mso_histogram[i][j] = 0;
            }
        }

        // Forever...
        while(true) {

            const vector<CochleaEvent> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            // Do asynchronous processing here
            for(auto &qi : *q) {

                // Get the real AER address
                address = qi.getAddress();

                // If debug flag enabled, print it out
                if (is_debug_flag == true) {
                    yDebug() << "Event received and decoded address: " << address;
                }
                
                // If the received event is valid
                if (address >= 0 && address < CochleaEvent::max_num_addresses) {
                    // Copy the address to the raw int32
                    out_event._coded_data = address;
                    // Copy the timestamp --> Is it necessary to pre-process?!!!!!!
                    out_event.stamp = qi.stamp;

                    // Copy the output event into the output queue
                    out_queue.push_back(out_event);   
                } else {
                    // Otherwise
                    yWarning() << "Not recognized event detected...";
                    address = -1;
                }

                // Sum up the event to the MSO matrix values only if the incoming
                // event is from MSO
                if(qi.auditory_model == 1){
                    if(qi.xso_type == 0){
                        // Get the freq channel id
                        int ch = qi.freq_chnn;
                        // Substract the MSO start channel to start the IDs in zero
                        ch = ch - mso_start_freq_channel;
                        // Get the neuron ID
                        int ne = qi.neuron_id;
                        // Increment the MSO matrix values in one
                        mso_histogram[ch][ne] = mso_histogram[ch][ne] + 1;
                    }
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
    rf.setDefaultConfigFile( "vCochleaEventsMapper.ini" );
    rf.configure( argc, argv );

    // Create the module
    vCochleaEventsMapper instance;
    return instance.runModule(rf);
}
