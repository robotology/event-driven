
#if defined MetavisionSDK_FOUND
    #include <metavision/sdk/driver/camera.h>
    #include <metavision/sdk/base/events/event_cd.h>
    using namespace Metavision;
#else
    #include <prophesee_driver/prophesee_driver.h>
    #include <prophesee_core/events/event_cd.h>
    using namespace Prophesee;
#endif

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <opencv2/opencv.hpp>
#include <yarp/cv/Cv.h>

using namespace ev;
using namespace yarp::os;

class atis3Bridge : public RFModule, public Thread {

private:

    Stamp yarpstamp;
    vWritePort output_port;
    Stamp graystamp;
    yarp::os::BufferedPort< yarp::sig::FlexImage > grayscale_port;
    Camera cam; // create the camera
    
    int counter_packets{0};
    int counter_events{0};
    static constexpr double period{1.0};

    std::mutex m;
    vector< vector<int32_t> > buffer;
    int buffer_size{0};
    int buffer_used{0};
    int b_sel{0};
    static constexpr void switch_buffer(int &buf_i) {buf_i = (buf_i + 1) % 2;};

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {

        if(rf.check("h") || rf.check("help")) {

            yInfo() << "Bridge to push ATIS gen3 camera and raw files to YARP";
            yInfo() << "--name <str>\t: internal port name prefix";
            yInfo() << "--buffer_size <int>\t: set initial maximum buffer size";
            yInfo() << "--file <str>\t: (optional) provide file path otherwise search for camera to connect";
            return false;
        }

        if((int)round(vtsHelper::vtsscaler) != 1000000)
            yWarning() << "USB ATIS Gen 3 typically has a clock period of 1 ms. You may need to compile event-driven "
                          "with a cmake parameter VLIB_CLOCK_PERIOD_NS=1000 for correct time scaling.";

        if(!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        //set the module name used to name ports
        setName((rf.check("name", Value("/atis3")).asString()).c_str());

        output_port.setWriteType(AE::tag);
        if(!output_port.open(getName("/AE:o"))) {
            yError() << "Could not open output port";
            return false;
        }

        if(!grayscale_port.open(getName("/img:o"))) {
            yError() << "Could not open image port";
            return false;
        } 

        //yarp::os::Network::connect(getName("/AE:o"), "/vPreProcess/AE:i", "fast_tcp");

        buffer_size = rf.check("buffer_size", Value(1000000)).asInt32();
        buffer.emplace_back(vector<int32_t>(buffer_size, 0));
        buffer.emplace_back(vector<int32_t>(buffer_size, 0));        

        if(rf.check("file")) {
            cam = Camera::from_file(rf.find("file").asString());
        } else {
            cam = Camera::from_first_available();
        }

        Biases &bias = cam.biases();
        int bias_pol = 0;
        if(rf.check("polarity"))
            bias_pol = rf.find("polarity").asInt32();
        if(rf.check("p"))
            bias_pol = rf.find("p").asInt32();
        if(bias_pol < 0) bias_pol = 1;
        if(bias_pol > 99) bias_pol = 99;
        
        int bias_sens = 0;
        if (rf.check("sensitivity"))
            bias_sens = rf.find("sensitivity").asInt32();
        if(rf.check("s"))
            bias_sens = rf.find("s").asInt32();
        if(bias_sens < 0) bias_sens = 1;
        if(bias_sens > 99) bias_sens = 99;

#if defined MetavisionSDK_FOUND
        I_LL_Biases* bias_control = bias.get_facility();
        std::map<std::string, int> bias_vals = bias_control->get_all_biases();
        yInfo() << "Default Biases:" << bias_vals["bias_diff_off"] << bias_vals["bias_diff_on"] << "[diff_off diff_on]";
        //for(auto i : bias_vals) yInfo() << i.first << i.second;
        if(bias_sens) {
            int diff_on  = (66 - 350) * 0.01 * bias_sens + 650 - 66;
            int diff_off = (66 + 200) * 0.01 * bias_sens + 100 - 66;
            bias_control->set("bias_diff_on", diff_on);
            bias_control->set("bias_diff_off", diff_off);
            bias_vals = bias_control->get_all_biases();
            yInfo() << "        Biases:" << bias_vals["bias_diff_off"] << bias_vals["bias_diff_on"] << "[diff_off diff_on]";
        }
        if(bias_pol) {
            yWarning() << "polarity bias not implemented for VGA";
        }
#else
        yInfo() << "Default Biases:" <<  bias.get_contrast_sensitivity() << bias.get_contrast_sensitivity_to_polarity() << "[Sensitivity PolaritySwing]";
        if(bias_sens) bias.set_contrast_sensitivity(bias_sens);
        if(bias_pol) bias.set_contrast_sensitivity_to_polarity(bias_pol);
        yInfo() << "        Biases:" <<  bias.get_contrast_sensitivity() << bias.get_contrast_sensitivity_to_polarity() << "[Sensitivity PolaritySwing]";
        cam.set_exposure_frame_callback(10, [this](timestamp ts, const cv::Mat &image){this->frameToPort(ts, image);});
#endif  

        cam.cd().add_callback([this](const EventCD *ev_begin, const EventCD *ev_end) {
            this->fill_buffer(ev_begin, ev_end);
        });

        const Geometry &geo = cam.geometry();
        yInfo() << "[" << geo.width() << "x" << geo.height() << "]";
        
        if(!cam.start()) {
            yError() << "Could not start the camera";
            return false;
        }

        return Thread::start();
    }

    double getPeriod() override
    {
        return period; //period of synchronous thread
    }

    bool interruptModule() override
    {
        //if the module is asked to stop ask the asynchronous thread to stop
        return Thread::stop();
    }

    void onStop() override
    {
        cam.stop();
        output_port.close();
    }

    //synchronous thread
    bool updateModule() override
    {
        yInfo() << counter_packets / period << "packets and"
                << (counter_events * 0.001) / period << "k events sent per second";
        counter_packets = counter_events = 0;

        if(!cam.is_running())
            Thread::stop();

        return Thread::isRunning();
    }

    void fill_buffer(const EventCD *begin, const EventCD *end) {

        // this loop allows us to get access to each event received in this callback
        static AE ae;

        m.lock();

        //see if we need a bigger buffer and auto reallocate. Should only be slow at the start of operation if the
        //buffers are set with a low value to begin with
        if(buffer_used + (end-begin)*2 > buffer[b_sel].size()) {
            buffer[b_sel].resize(buffer[b_sel].size() * 4);
            yInfo() << "Buffer" << b_sel << "resized to" << buffer[b_sel].size();
        }

        //fill up the buffer that will be sent over the port in the other thread
        for (const EventCD *ev = begin; ev != end; ++ev) {
            ae.x = ev->x; ae.y = ev->y; ae.polarity = ev->p;
            buffer[b_sel][buffer_used++] = ev->t;
            buffer[b_sel][buffer_used++] = ae._coded_data;
        }

        m.unlock();

    }

    //using ExposureFrameCallback = std::function<void(Prophesee::timestamp, const cv::Mat &)>;
    void frameToPort(timestamp ts, const cv::Mat &image)
    {
        static double val1 = log(1000);
        static double val2 = log(100000);
        cv::Mat work, mask, output; 
        image.convertTo(work, CV_32F);
        cv::log(work, work);

        mask = work < val1; work.setTo(val1, mask);
        mask = work > val2; work.setTo(val2, mask);
        work -= val1;
        
        work.convertTo(output, CV_8U, 255.0/(val2-val1));
        output = 255 - output;

        grayscale_port.prepare().copy(yarp::cv::fromCvMat<yarp::sig::PixelMono>(output));
        graystamp.update();
        grayscale_port.setEnvelope(graystamp);
        grayscale_port.write();
    }

    //asynchronous thread run forever
    void run() override
    {
        while(!Thread::isStopping()) {

            //if we have data to send, do so, otherwise we are just going to wait for 1 ms
            if(buffer_used > 0) {

                //switch buffers so the callback can keep filling the second buffer while we are sending
                m.lock();
                vector<int32_t> &current_buffer = buffer[b_sel];
                int n_to_write = buffer_used;
                switch_buffer(b_sel);
                buffer_used = 0;
                m.unlock();

                //send the data in the first buffer
                yarpstamp.update();
                output_port.write(current_buffer, yarpstamp, n_to_write);
                counter_packets++;
                counter_events += n_to_write / 2;

            } else {
                Time::delay(0.001);
            }
        }

    }
};

int main(int argc, char * argv[])
{

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "atis3-bridge.ini" );
    rf.configure( argc, argv );

    /* create the module */
    atis3Bridge instance;
    return instance.runModule(rf);
}
