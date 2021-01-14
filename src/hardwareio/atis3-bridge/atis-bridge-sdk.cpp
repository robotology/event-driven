#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <yarp/os/all.h>
#include <event-driven/all.h>
using namespace ev;
using namespace yarp::os;

class atis3Bridge : public RFModule, public Thread {

private:

    vWritePort output_port;
    Metavision::Camera cam; // create the camera
    Stamp yarpstamp;
    int counter_packets{0};
    int counter_events{0};
    static constexpr double period{1.0};

    static constexpr void switch_buffer(int &buf_i) {buf_i = (buf_i + 1) % 2;};
    int b_sel{0};
    deque<AE> buffer[2];
    std::mutex m;

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {

        if(rf.check("h") || rf.check("help")) {

            yInfo() << "Bridge to push ATIS gen3 camera and raw files to YARP";
            yInfo() << "--name <str>\t: internal port name prefix";
            yInfo() << "--file <str>\t: (optional) provide file path otherwise search for camera to connect";
            return false;
        }

        if(!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        //set the module name used to name ports
        setName((rf.check("name", Value("/atis3")).asString()).c_str());

        if(!output_port.open(getName("/AE:o"))) {
            yError() << "Could not open output port";
            return false;
        }
        yarp::os::Network::connect(getName("/AE:o"), "/vPreProcess/AE:i", "fast_tcp");

        if(rf.check("file")) {
            cam = Metavision::Camera::from_file(rf.find("file").asString());
        } else {
            cam = Metavision::Camera::from_first_available();
        }

        cam.cd().add_callback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
            this->fill_buffer(ev_begin, ev_end);
        });

        if(!cam.start())
        {
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
        //if the module is asked to stop ask the asynchrnous thread to stop
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

    void fill_buffer(const Metavision::EventCD *begin, const Metavision::EventCD *end) {

        // this loop allows us to get access to each event received in this callback
        AE ae;

        m.lock();
        for (const Metavision::EventCD *ev = begin; ev != end; ++ev) {
            ae.x = ev->x;
            ae.y = ev->y;
            ae.polarity = ev->p;
            ae.stamp = ev->t;

            buffer[b_sel].push_back(ae);

        }
        m.unlock();

    }

    //asynchronous thread run forever
    void run() override
    {
        while(!Thread::isStopping()) {
            deque<AE> &current_buffer = buffer[b_sel];
            m.lock();
            switch_buffer(b_sel);
            m.unlock();

            if (!current_buffer.empty()) {
                yarpstamp.update();
                output_port.write(current_buffer, yarpstamp);
                counter_packets++;
                counter_events += current_buffer.size();
                current_buffer.clear();
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
