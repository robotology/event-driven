/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
#include <vector>
#include "event-driven/core.h"
#include <yarp/cv/Cv.h>
using namespace ev;
using namespace yarp::os;

class atis3Bridge : public RFModule, public Thread {

private:

    yarp::os::Port output_port;
    Stamp yarpstamp;
    Stamp graystamp;
    yarp::os::BufferedPort< yarp::sig::FlexImage > grayscale_port;

    Camera cam; // create the camera
    
    int counter_packets{0};
    int counter_events{0};
    static constexpr double period{1.0};

    std::mutex m;
    std::vector< ev::packet<TAE> > buffer;
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

        if(!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        try {
            if (rf.check("file")) {
                cam = Camera::from_file(rf.find("file").asString());
            } else {
                cam = Camera::from_first_available();
            }
        } catch(const std::exception& e) {
            yError() << "No cameras connected";
            return false;
        }

        if ((int)round(ev::vtsscaler) != 1000000)
            yWarning() << "ATIS USB typically has a clock period of 1 ms. You may need to compile event-driven "
                          "with a cmake parameter VLIB_CLOCK_PERIOD_NS=1000 for correct time scaling.";

        //set the module name used to name ports
        setName((rf.check("name", Value("/atis3")).asString()).c_str());

        if(!output_port.open(getName("/AE:o"))) {
            yError() << "Could not open output port";
            return false;
        }

        if(!grayscale_port.open(getName("/img:o"))) {
            yError() << "Could not open image port";
            return false;
        } 

        //yarp::os::Network::connect(getName("/AE:o"), "/vPreProcess/AE:i", "fast_tcp");

        buffer.emplace_back();
        buffer.emplace_back();        

        Biases &bias = cam.biases();
#if defined MetavisionSDK_FOUND
        //it seems like the ability ot set the camera sensitivity "easily" has been removed, and relegated to
        //setting the entire biases file. perhaps if the HAL is included and setup correctly then
        // the flag I_HL_BIASES_FACILITY_AVAILABLE is set and we can use the "constrast sensitivity".
#else
        int bias_pol = 0;
        if(rf.check("polarity"))
            bias_pol = rf.find("polarity").asInt();
        if(rf.check("p"))
            bias_pol = rf.find("p").asInt();
        
        int bias_sens = 0;
        if (rf.check("sensitivity"))
            bias_sens = rf.find("sensitivity").asInt();
        if(rf.check("s"))
            bias_sens = rf.find("s").asInt();

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

        TAE tae;
        // this loop allows us to get access to each event received in this callback
        m.lock();
        //fill up the buffer that will be sent over the port in the other thread
        for (const EventCD *ev = begin; ev != end; ++ev) {
            tae.ts = ev->t; tae.x = ev->x; tae.y = ev->y; tae.p = ev->p;
            buffer[b_sel].push_back(tae);
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
        double tic = yarp::os::Time::now();
        while(!Thread::isStopping()) {

            //if we have data to send, do so, otherwise we are just going to wait for 1 ms
            ev::packet<TAE> &current_buffer = buffer[b_sel];
            if(current_buffer.size() > 0) {

                //switch buffers so the callback can keep filling the second buffer while we are sending
                m.lock();
                switch_buffer(b_sel);
                m.unlock();

                //send the data in the first buffer
                current_buffer.duration(yarp::os::Time::now()-tic);
                tic += current_buffer.duration();
                yarpstamp.update();
                output_port.setEnvelope(yarpstamp);
                output_port.write(current_buffer);
                counter_packets++;
                counter_events += current_buffer.size();
                current_buffer.clear();

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
