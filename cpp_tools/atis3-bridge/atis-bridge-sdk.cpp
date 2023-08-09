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
#include <yarp/cv/Cv.h>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>

#include "event-driven/core.h"
#include "event-driven/vis.h"

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
    bool record_mode{false};

    ev::vNoiseFilter nf;

    std::mutex m;
    std::condition_variable signal;
    std::vector< ev::packet<AE> > buffer;
    int buffer_size{0};
    int buffer_used{0};
    int b_sel{0};
    double limit{-1};
    static constexpr void switch_buffer(int &buf_i) {buf_i = (buf_i + 1) % 2;};

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {

        if(rf.check("h") || rf.check("help")) {

            yInfo() << "Bridge to push ATIS gen3 camera and raw files to YARP";
            yInfo() << "--name <str>\t: internal port name prefix";
            yInfo() << "--buffer_size <int>\t: set initial maximum buffer size";
            yInfo() << "--file <str>\t: (optional) provide file path otherwise search for camera to connect";
            yInfo() << "--limit <int>\t: (optional) provide a hard limit on event rate (in 10^6 events/s)";
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

        record_mode = rf.check("record_mode") && 
                      rf.check("record_mode", Value(true)).asBool();

        limit = rf.check("limit", Value(-1)).asFloat64();
        if(limit < 0) limit  = DBL_MAX;
        else          limit *= 1e6;

        //yarp::os::Network::connect(getName("/AE:o"), "/vPreProcess/AE:i", "fast_tcp");

        buffer.emplace_back();
        buffer.emplace_back();        

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

        double nf_param = 0.0;
        if(rf.check("filter")) nf_param = rf.find("filter").asFloat64();
        if(rf.check("f")) nf_param = rf.find("f").asFloat64();
        if(nf_param > 0.0) 
        {
            yInfo() << "[FILTER] ON. Maximum 1 event per pixel per" << nf_param << "seconds";
            nf.initialise(geo.width(), geo.height());
            nf.use_temporal_filter(nf_param);
        }

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
        signal.notify_one();
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

    void fill_buffer(const EventCD *begin, const EventCD *end) 
    {
        std::unique_lock<std::mutex> lk(m);
        //fill up the buffer that will be sent over the port in the other thread
        AE ae;
        if (nf.active()) {
            for (const EventCD *ev = begin; ev != end; ++ev) {
                if(nf.check(ev->x, ev->y, ev->p, ev->t * 0.000001)) {
#if ENABLE_TS
                    ae.ts = ev->t;
#endif
                    ae.x = ev->x; ae.y = ev->y; ae.p = ev->p;
                    buffer[b_sel].push_back(ae);
                }
            }
        } else {
            for (const EventCD *ev = begin; ev != end; ++ev) {
#if ENABLE_TS
                ae.ts = ev->t;
#endif
                ae.x = ev->x; ae.y = ev->y; ae.p = ev->p;
                buffer[b_sel].push_back(ae);
            }
        }
        lk.unlock();
        signal.notify_one();

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
        const static constexpr double packet_time = 0.004;
        double tic = yarp::os::Time::now();
        while(true) {
            // wait for data and then switch buffers so the callback can keep
            // filling the second buffer while we are sending
            ev::packet<AE> &current_buffer = buffer[b_sel];
            std::unique_lock<std::mutex> lk(m);
            signal.wait(lk, [this] { return buffer[b_sel].size() > 0 || Thread::isStopping(); });
            switch_buffer(b_sel);
            lk.unlock();

            if(Thread::isStopping()) break;

            // send the data in the first buffer
            current_buffer.duration(yarp::os::Time::now() - tic);
            tic += current_buffer.duration();
            yarpstamp.update();
            if(current_buffer.size() / current_buffer.duration() < limit) 
            {
                output_port.setEnvelope(yarpstamp);
                output_port.write(current_buffer);
            }
            counter_packets++;
            counter_events += current_buffer.size();
            current_buffer.clear();

            if (record_mode && current_buffer.duration() < packet_time)
                Time::delay(packet_time - current_buffer.duration());
        }

    }
};

int main(int argc, char * argv[])
{

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    /* create the module */
    atis3Bridge instance;
    return instance.runModule(rf);
}
