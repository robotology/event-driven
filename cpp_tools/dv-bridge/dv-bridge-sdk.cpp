#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <yarp/os/all.h>
#include <yarp/cv/Cv.h>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <sstream>

#include "event-driven/core.h"
#include "event-driven/vis.h"

using namespace ev;
using namespace yarp::os;


class dvbridge : public RFModule, public Thread {

private:

    yarp::os::Port output_port;
    Stamp yarpstamp;

    dv::io::camera::DVXplorerM cam{}; // create and open the DVXplorer Micro camera
    std::optional<cv::Size> resolution = cam.getEventResolution();
    bool bgaFilter{false};
    int bgaDuration{2000};
    
    
    int counterPackets{0};
    int counterEvents{0};
    static constexpr double period{0.2};
    bool recordMode{false};

    ev::vNoiseFilter nf;

    std::mutex m;
    std::condition_variable signal;
    std::vector< ev::packet<AE> > buffer;
    int bufferSize{0};
    int bufferUsed{0};
    int bSel{0};
    double limit{-1};
    static constexpr void switch_buffer(int &buf_i) {buf_i = (buf_i + 1) % 2;};
    double clock_time{0};

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {

        if(rf.check("h") || rf.check("help")) {

            yInfo() << "Bridge to push Inivation DVXplorer Micro camera events to YARP.";
            yInfo() << "Usage: dvbridge [options]";
            yInfo() << "Options:";
            yInfo() << "\t--threshold <int>\t: (optional) Set threshold intensity variation for event output (default: 9, range 0-17)";
            yInfo() << "\t--bgafilter <int>\t: (optional) Enable background activity filter with duration in microseconds (default: 2000)";
            yInfo() << "\t--filter <float>\t: (optional) Enable event-driven temporal noise filter (seconds per event per pixel)";
            yInfo() << "\t--limit <float>   \t: (optional) Maximum events per second in buffer (in millions, default: unlimited)";
            yInfo() << "\t--name <str>      \t: (optional) Module name used to name the YARP ports (default: /dv)";
            yInfo() << "\t--help        \t\t: Show this help message and exit";
            yInfo() << "";
            yInfo() << "Example:";
            yInfo() << "\tdv-bridge-sdk --threshold 10 --bgafilter 3000 --filter 0.01 --limit 2 --name /mydv";
            return false;
        }

        if(!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        
        limit = rf.check("limit", Value(-1)).asFloat64();
        if(limit < 0) limit  = DBL_MAX;
        else          limit *= 1e6;

        buffer.emplace_back();
        buffer.emplace_back();

        if (rf.check("threshold")) {
          int threshold = rf.check("threshold", Value(9)).asInt8();
          cam.setContrastThresholdOn(threshold);
          cam.setContrastThresholdOff(threshold);
          yInfo() << "[THRESHOLD] The DVXplorer Micro threshold is set to"<< threshold;
        }
      
        if (rf.check("bgafilter")) {
          bgaFilter = true;
          bgaDuration = rf.check("bgafilter", Value(2000)).asInt16();
          yInfo() << "[BGA_FILTER] The DVXplorer background activity filter duration is"<< bgaDuration;
        }
      
        double nf_param = 0.0;
        if(rf.check("filter")) nf_param = rf.find("filter").asFloat64();
        if(rf.check("f")) nf_param = rf.find("f").asFloat64();
        if(nf_param > 0.0) 
        {
            yInfo() << "[FILTER] ON. Maximum 1 event per pixel per" << nf_param << "seconds";
            nf.initialise(resolution->width, resolution->height);
            nf.use_temporal_filter(nf_param);
        }

        //set the module name used to name ports
        setName((rf.check("name", Value("/dv")).asString()).c_str());


        //automatically assign port numbers
        std::stringstream ss;
        ss.str(""); ss << getName() << "/AE:o";
        if(yarp::os::Network::exists(ss.str())) {
            int port_number = 1; 
            do {
                port_number++;
                ss.str(""); ss << getName() << "-" << port_number << "/AE:o";
            } while(yarp::os::Network::exists(ss.str()));
        }

        if(!output_port.open(ss.str())) {
            yError() << "Could not open output port";
            return false;
        }

        if(!cam.isRunning()) {
            yError() << "Could not start the camera";
            return false;
        }
      
        clock_time = yarp::os::Time::now();

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
        output_port.close();
        signal.notify_one();
    }

    //synchronous thread
    bool updateModule() override
    {
        //perform synchronisation between CPU and camera clock
        double oos = yarp::os::Time::now() - clock_time;
        static int i = 0;
        if(i++ < 5) {
            clock_time += oos*0.5;
        } else {
            clock_time += oos*0.005;
        }

        //output some nice info
        if(i % (int)(1/period) == 0) {
        yInfo() << counterPackets << "packets and"
                << (counterEvents * 0.001) << "k events sent per second";
        counterPackets = counterEvents = 0;
        }

        if(!cam.isRunning())
            Thread::stop();

        return Thread::isRunning();
    }

    void fill_buffer(const auto &events) 
    {
        std::unique_lock<std::mutex> lk(m);
        //fill up the buffer that will be sent over the port in the other thread
        AE ae;
        if (nf.active()) {
            for (const auto& ev: events) {
                if(nf.check(ev.x(), ev.y(), ev.polarity(), ev.timestamp() * 0.000001)) {
                    ae.x = ev.x(); ae.y = ev.y(); ae.p = ev.polarity();
                    buffer[bSel].push_back(ae);
                }
            }
        } else {
            for (const auto& ev: events) {
                ae.x = ev.x(); ae.y = ev.y(); ae.p = ev.polarity();
                buffer[bSel].push_back(ae);
            }
        }
        dv::Duration dur = events.duration();
        buffer[bSel].duration(static_cast<double>(dur.count())*1e-6 + buffer[bSel].duration());
        lk.unlock();
        signal.notify_one();

    }



    //asynchronous thread run forever
    void run() override
    {
      const dv::Duration backgroundActivityDuration = dv::Duration(bgaDuration);
        dv::noise::BackgroundActivityNoiseFilter<> BGAFilter(resolution.value(),backgroundActivityDuration);
        while(cam.isRunning()) {
            // wait for data and then switch buffers so the callback can keep
            // filling the second buffer while we are sending
          
            if (auto events = cam.getNextEventBatch(); events.has_value()) {
              if (bgaFilter){
                BGAFilter.accept(*events);
                dv::EventStore BGAFiltered = BGAFilter.generateEvents();
                if (BGAFiltered.isEmpty()) {
                    continue; // skip empty batches
                  }
                fill_buffer(BGAFiltered);
                }
              else {
                fill_buffer(*events);
                } 
              ev::packet<AE> &current_buffer = buffer[bSel];
              std::unique_lock<std::mutex> lk(m);
              signal.wait(lk, [this] { return buffer[bSel].size() > 0 || Thread::isStopping(); });
              switch_buffer(bSel);
              buffer[bSel].duration(0.0);
              lk.unlock();

              if(Thread::isStopping()) break;

              // send the data in the first buffer
              clock_time += current_buffer.duration();
              yarpstamp.update(clock_time);
              if(current_buffer.size() / current_buffer.duration() < limit) 
              {
                output_port.setEnvelope(yarpstamp);
                output_port.write(current_buffer);
              }
              counterPackets++;
              counterEvents += current_buffer.size();
              current_buffer.clear();
              }
          }
      }
    
    void make_plot(double seconds)
    {
        static const int NX = 40;
        static const int NY = 50;
        static const int xx = 10;
        static const int yx = 1;

        static cv::Mat img(2*NY*yx, NX*xx, CV_8U);

        static std::list<int> buffer;
        double ms = seconds*1000;
        if(ms > NY) ms = NY; if(ms < -NY) ms = -NY;
        buffer.push_back((int)((NY - ms)*yx));
        while(buffer.size() > NX) buffer.pop_front();

        img.setTo(0); int i = 1;
        auto p1 = buffer.begin(); auto p2 = buffer.begin(); p2++;
        for(; p2 != buffer.end(); p1++, p2++, i++) {
            cv::line(img, {xx*(i-1), *p1}, {xx*i, *p2}, 255, 1);
        }

        cv::line(img, {0, NY*yx}, {NX*xx, NY*yx}, 128);
        std::stringstream ss;
        ss.str(""); ss << NY << "ms";
        cv::putText(img, ss.str(), {2, 20}, cv::FONT_HERSHEY_PLAIN, 1.0, 255);
        ss.str(""); ss << -NY << "ms";
        cv::putText(img, ss.str(), {2, img.rows - 2}, cv::FONT_HERSHEY_PLAIN, 1.0, 255);

        cv::imshow("clock synch", img);
        cv::waitKey(10);

    }
};

int main(int argc, char * argv[])
{

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    /* create the module */
    dvbridge instance;
    return instance.runModule(rf);
}