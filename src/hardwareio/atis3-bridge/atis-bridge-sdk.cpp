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
        //yarp::os::Network::connect(getName("/AE:o"), "/vPreProcess/AE:i", "fast_tcp");

        buffer_size = rf.check("buffer_size", Value(1000000)).asInt();
        buffer.emplace_back(vector<int32_t>(buffer_size, 0));
        buffer.emplace_back(vector<int32_t>(buffer_size, 0));

        if(rf.check("file")) {
            cam = Metavision::Camera::from_file(rf.find("file").asString());
        } else {
            cam = Metavision::Camera::from_first_available();
        }

        cam.cd().add_callback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
            this->fill_buffer(ev_begin, ev_end);
        });

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

    void fill_buffer(const Metavision::EventCD *begin, const Metavision::EventCD *end) {

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
        for (const Metavision::EventCD *ev = begin; ev != end; ++ev) {

            ae.x = ev->x; ae.y = ev->y; ae.polarity = ev->p;
            buffer[b_sel][buffer_used++] = ev->t;
            buffer[b_sel][buffer_used++] = ae._coded_data;

        }

        m.unlock();

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
