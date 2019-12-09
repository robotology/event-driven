
#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace ev;
using namespace yarp::os;

class portNameReporter : public PortReport
{
public:

    string incoming_port_name;

    portNameReporter()
    {
        incoming_port_name = "/dummy_port_name/AE:i";
    }

    void report(const PortInfo &info) override
    {
        yInfo() << "Incoming connection detected";
        yInfo() << info.incoming << info.created << info.sourceName;
        if(info.incoming && info.created)
            incoming_port_name = info.sourceName;
    }
};

class binaryDumper : public RFModule, public Thread {

private:

    portNameReporter port_name_reader;
    vReadPort< vector<int32_t> > input_port;
    std::ofstream info_dumper;
    std::ofstream event_dumper;
    int packets_saved;

    bool save_first;

    double first_zynq_time;
    double first_cpu_time;
    double total_zynq_dt;
    double last_cpu_time;

public:

    binaryDumper() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/custom-dumper")).asString()).c_str());

        //open io ports
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }
        input_port.setReporter(port_name_reader);

        char char_buffer[50];
        char * cwd = getcwd(char_buffer, 50);
        if(!cwd) {
            yError() << "Cannot get default path as the CWD";
            return false;
        }
        string default_path(char_buffer);

        string path = rf.check("path", Value(default_path)).asString();
        string event_filename = path + "/binaryevents.log";
        string info_filename = path + "/info.log";

        std::ifstream checker;
        checker.open(event_filename, std::ios_base::binary | std::ios_base::in);
        if(checker.good()) {
            yError() << event_filename << "exists.";
            return false;
        }
        checker.close();

        checker.open(info_filename);
        if(checker.good()) {
            yError() << info_filename << "exists.";
            return false;
        }
        checker.close();


        event_dumper.open(event_filename, std::ios_base::trunc | std::ios_base::binary | std::ios_base::out);
        if(!event_dumper.is_open()) {
            yError() << "Could not open " << event_filename << "for writing.";
            return false;
        }

        info_dumper.open(info_filename, std::ios_base::trunc | std::ios_base::out);
        if(!info_dumper.is_open()) {
            yError() << "Could not open " << info_filename << "for writing.";
            return false;
        }

        info_dumper << "Type: Binary;" << std::endl;
        info_dumper << std::fixed << std::setprecision(6);

        save_first = true;
        packets_saved = 0;
        std::cout << std::endl;

        //start the asynchronous and synchronous threads
        return Thread::start();
    }

    virtual double getPeriod()
    {
        return 1.0; //period of synchrnous thread
    }

    bool interruptModule()
    {
        std::cout << std::endl;
        //if the module is asked to stop ask the asynchrnous thread to stop
        return Thread::stop();
    }

    void onStop()
    {

        //when the asynchrnous thread is asked to stop, close ports and do
        //other clean up
        input_port.close();
    }

    void threadRelease()
    {
        yInfo() << "zynq/cpu skew is" << (last_cpu_time - first_cpu_time) - total_zynq_dt;

        if(!save_first) {
            //save last only if we saved first
            info_dumper << "[" << first_cpu_time  + total_zynq_dt  << "] "
                        << port_name_reader.incoming_port_name
                        << " [disconnected]" << std::endl;
        }

        info_dumper.close();
        event_dumper.close();
    }

    //synchronous thread
    virtual bool updateModule()
    {

        //add any synchronous operations here, visualisation, debug out prints
        std::cout << "\r" << packets_saved << " packets saved. "
                  << input_port.queryunprocessed() << " packets in queue."
                  << "                                                    ";
        std::cout.flush();

        return Thread::isRunning();
    }

    //asynchronous thread run forever
    void run()
    {
        Stamp yarpstamp;
        total_zynq_dt = 0;

        while(true) {

            //get the first packet (blocking read)
            const vector<int32_t> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            //only when the first packet is read do our first initialisation
            if(save_first) {
                //save first stamps (zynq and local cpu)
                first_zynq_time = yarpstamp.getTime();
                first_cpu_time = yarp::os::Time::now();

                //do a small correction so we are aligned to the first event
                //in the packet, and not the last
                double correction_factor = (*q)[q->size() - 2] - (*q)[0];
                if(correction_factor < 0) correction_factor += vtsHelper::max_stamp;
                correction_factor *= vtsHelper::tsscaler;
                yInfo() << "Correction factor" << correction_factor;
                first_zynq_time -= correction_factor;
                first_cpu_time -= correction_factor;

                //dump our corrected cpu time
                info_dumper << "[" << first_cpu_time  << "] "
                            << port_name_reader.incoming_port_name
                            << " [connected]" << std::endl;
                save_first = false;
            }

            //write the event-stream to binary file
            event_dumper.write((const char *)q->data(), q->size() * sizeof(int32_t));

            //update the total time of the data-stream
            //from zynq clock (irrespective of any "back-load" of packets to save)
            total_zynq_dt = yarpstamp.getTime() - first_zynq_time;
            last_cpu_time = Time::now();

            //count our progress
            packets_saved++;
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
    binaryDumper instance;
    return instance.runModule(rf);
}
