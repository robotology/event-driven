
#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

#define IS_DEBUG 1

#define IS_NAS_MODEL 0
#define IS_SOC_MODEL 1

#define IS_MSO_MODEL 0
#define IS_LSO_MODEL 1

class vCochleaEventsMapper : public RFModule, public Thread {

private:

    vReadPort< vector<CochleaEvent> > input_port;
    vWritePort output_port;

    bool example_flag;

    // NAS parameters
    int nas_num_frequency_channels;
    int nas_polarity_type;
    int nas_mono_stereo;

    // MSO parameters
    int mso_start_freq_channel;
    int mso_end_freq_channel;
    int mso_num_neurons_per_channel;

    // LSO parameters
    int lso_start_freq_channel;
    int lso_end_freq_channel;
    int lso_num_neurons_per_channel;

    // Addresses constants
    const int offset_stereo_addresses = nas_num_frequency_channels * nas_polarity_type * (nas_mono_stereo - 1);
    const int offset_nas_addresses = nas_num_frequency_channels * nas_polarity_type * nas_mono_stereo;
    const int offset_mso_addresses = (mso_end_freq_channel - mso_start_freq_channel + 1) * mso_num_neurons_per_channel;

public:

    vCochleaEventsMapper() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/vCochleaEventsMapper")).asString()).c_str());

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

        // Read NAS parameters
        int default_nas_num_frequency_channels = 32;
        nas_num_frequency_channels = rf.check("nasNumFreqChann", 
                                    Value(default_nas_num_frequency_channels)).asInt();
        yInfo << "Setting nas_num_frequency_channels parameter to: " << nas_num_frequency_channels;

        int default_nas_polarity_type = 2;
        nas_polarity_type = rf.check("nasPolType",
                                    Value(default_nas_polarity_type)).asInt();
        yInfo << "Setting nas_polarity_type parameter to: " << nas_polarity_type;

        int default_nas_mono_stereo = 2;
        nas_mono_stereo = rf.check("nasMonoStereo",
                                    Value(default_nas_mono_stereo)).asInt();
        yInfo << "Setting nas_mono_stereo parameter to: " << nas_mono_stereo;

        // Read MSO parameters
        int default_mso_start_freq_channel = 13;
        mso_start_freq_channel = rf.check("msoStartFreqChann",
                                    Value(default_mso_start_freq_channel)).asInt();
        yInfo << "Setting mso_start_freq_channel parameter to: " << mso_start_freq_channel;

        int default_mso_end_freq_channel = 16;
        mso_end_freq_channel = rf.check("msoEndFreqChann",
                                    Value(default_mso_end_freq_channel)).asInt();
        yInfo << "Setting mso_end_freq_channel parameter to: " << mso_end_freq_channel;

        int default_mso_num_neurons_per_channel = 16;
        mso_num_neurons_per_channel = rf.check("msoNumNeuronsPerChann",
                                    Value(default_mso_num_neurons_per_channel)).asInt();
        yInfo << "Setting mso_num_neurons_per_channel parameter to: " << mso_num_neurons_per_channel;


        // Read LSO parameters
        int default_lso_start_freq_channel = 5;
        lso_start_freq_channel = rf.check("lsoStartFreqChann",
                                    Value(default_lso_start_freq_channel)).asInt();
        yInfo << "Setting lso_start_freq_channel parameter to: " << lso_start_freq_channel;

        int default_lso_end_freq_channel = 8;
        lso_end_freq_channel = rf.check("lsoEndFreqChann",
                                    Value(default_lso_end_freq_channel)).asInt();
        yInfo << "Setting lso_end_freq_channel parameter to: " << lso_end_freq_channel;

        int default_lso_num_neurons_per_channel = 1;
        lso_num_neurons_per_channel = rf.check("lsoNumNeuronsPerChann",
                                    Value(default_lso_num_neurons_per_channel)).asInt();
        yInfo << "Setting lso_num_neurons_per_channel parameter to: " << lso_num_neurons_per_channel;

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
        yInfo << "Interrupting the module: stopping thread..."
        return Thread::stop();
    }

    void onStop()
    {
        //when the asynchrnous thread is asked to stop, close ports and do
        //other clean up
        yInfo << "Stopping the module..."
        yInfo << "Closing input port..."
        input_port.close();
        yInfo << "Closing output port..."
        output_port.close();
        yInfo << "Module has been closed!"
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
        deque<CochleaEvent> out_queue;
        CochleaEvent out_event;
        int address = 0;

        while(true) {

            const vector<CochleaEvent> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            //do asynchronous processing here
            for(auto &qi : *q) {

                //here you could try modifying the data of the event before
                //pushing to the output q

                if(qi.auditory_model == IS_NAS_MODEL) {
                    // Is NAS event
                    address = (qi.freq_chnn * nas_polarity_type) + qi.polarity + (qi.channel * offset_stereo_addresses);
                    yInfo << "NAS event detected with address: " << address;
                } else if(qi.auditory_model == IS_SOC_MODEL) {
                    if (qi.xso_type == IS_MSO_MODEL) {
                        // Is MSO event
                        address = offset_nas_addresses + qi.neuron_id + ((qi.freq_chnn - mso_start_freq_channel) * mso_num_neurons_per_channel);
                        yInfo << "MSO event detected with address: " << address;
                    } else if(qi.xso_type == IS_LSO_MODEL) {
                        // Is LSO event
                        address = offset_nas_addresses + offset_mso_addresses + qi.neuron_id + ((qi.freq_chnn - lso_start_freq_channel) * lso_num_neurons_per_channel);
                        yInfo << "LSO event detected with address: " << address;
                    } else {
                        // Otherwise
                        yWarning << "Not recognized event detected...";
                        address = -1;
                    }
                } else {
                    // Otherwise
                    yWarning << "Not recognized event detected...";
                    address = -1;
                }
                
                // If the received event is valid
                if (address >= 0) {
                    // Copy the address to the raw int32
                    out_event._cochleaei = address;
                    // Copy the timestamp --> Is it necessary to pre-process?!!!!!!
                    out_event.stamp = qi.stamp;

                    // Copy the output event into the output queue
                    out_queue.push_back(out_event);   
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
    rf.setDefaultConfigFile( "sample_module.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vCochleaEventsMapper instance;
    return instance.runModule(rf);
}
