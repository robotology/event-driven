/*
 *   Copyright (C) 2022 Event-driven Perception for Robotics
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

#include <yarp/os/all.h>
#include <event-driven/core.h>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace ev;
using namespace yarp::os;

class exampleModule : public RFModule {

private:

    //yarp ports
    ev::window<AE> input_port;
    ev::BufferedPort<AE> output_port;

    //threads
    std::mutex m;
    std::condition_variable signal;
    std::thread event_thread;
    std::thread proc_thread;
    double current_ts{0.0}, previous_ts{0.0};

    //internal data
    cv::Mat my_eros;
    bool example_flag;
    double example_parameter;

public:

    exampleModule() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        if(rf.check("h") || rf.check("help")) 
        {
            yInfo() << "help message";
            yInfo() << "variable 1 <int> : description";
        }

        /* initialize yarp network */
        if (!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        //set the module name used to name ports
        setName((rf.check("name", Value("/example-module")).asString()).c_str());

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
        double default_value = 0.1;
        example_parameter = rf.check("example_parameter",
                                     Value(default_value)).asFloat64();

        //do any other set-up required here
        cv::namedWindow(getName(), cv::WINDOW_NORMAL);
        cv::resizeWindow(getName(), cv::Size(640, 480));
        event_thread = std::thread([this]event_process);
        proc_thread = std::thread([this]secondary_process);

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
        input_port.close();
        event_thread.join();
        proc_thread.join();
        return Thread::stop();
    }

    //synchronous thread (threaded) critical function
    virtual bool updateModule()
    {
        cv::imshow(getName(), my_eros);
        cv::waitKey(1);

        //add any synchronous operations here, visualisation, debug out prints
        return Thread::isRunning();
    }

    //critical (threaded) function
    void event_process()
    {
        while(!isStopping()) 
        {
            ev::info stats = input_port.readAll(true);

            std::unique_lock<std::mutex> lk(m);
            for(auto &v : input_port)
                ev::eros.update();
            current_ts = stats.timestamp;
            lk.unlock();
            signal.notify_one();
        }
    }

    //critical (threaded) function
    void secondary_process() 
    {
        while(!isStopping()) 
        {
            //get the data structure
            std::unique_lock<std::mutex> lk(m);
            signal.wait(lk, [this](return current_ts > previous_ts || isStopping()););
            if(isStopping()) break;
            previous_ts = current_ts;
            ev::eros.getSurface().copyTo(my_eros);
            lk.unlock();

            //further processing

            //event-driven output if necessary
            ev::packet<AE> &output_packet = output_port.prepare();
            output_packet.clear();
            AE ae;
            output_packet.duration();
            output_packet.timestamp();
            output_packet.push_back(ae);
            output_port.write();
        }
    }
};

int main(int argc, char * argv[])
{
    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    /* create the module */
    exampleModule instance;
    return instance.runModule(rf);
}
