/*
 *   Copyright (C) 2020 Event-driven Perception for Robotics
 *   Author: dgutierrez@atc.us.es
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
#include <event-driven/all.h>

using namespace ev;
using namespace yarp::os;

class vSpinnakerEventsMapper : public RFModule, public Thread {

private:
    // Input port: type AE
    vReadPort< vector<AE> > input_port;
    // Output port for pure tones classification: type AE
    vWritePort output_port_tones;
    // Output port for sound source localization: type AE
    vWritePort output_port_soundsource;

    // Debug flag
    // If activated, the module shows info about the classification in the terminal
    bool is_debug_flag;

    // Number of tones to be classified
    // ----------------------------------------------------------------------------
    // WARNING!! This number must be the same as the number of output neurons used
    // in SpiNNaker tones out population
    // ----------------------------------------------------------------------------
    int number_tones_output_neurons;

    // Number of sound sources to be classified
    // ----------------------------------------------------------------------------
    // WARNING!! This number must be the same as the number of output neurons used
    // in SpiNNaker sound sources
    // ----------------------------------------------------------------------------
    int number_sound_source_neurons;

    // Period
    double update_period;

public:

    vSpinnakerEventsMapper() {}

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        yInfo() << "Configuring the module...";

        // Set the module name used to name ports
        setName((rf.check("name", Value("/vSpinnakerEventsMapper")).asString()).c_str());

        // Open input port
        yInfo() << "Opening input port...";
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }

        // Openn output tones port
        yInfo() << "Opening output tones port...";
        output_port_tones.setWriteType(AE::tag);
        if(!output_port_tones.open(getName() + "/tones:o")) {
            yError() << "Could not open output tones port";
            return false;
        }
        // Open output localization port
        yInfo() << "Opening output soundsource port...";
        output_port_soundsource.setWriteType(AE::tag);
        if(!output_port_soundsource.open(getName() + "/soundsource:o")) {
            yError() << "Could not open output localization port";
            return false;
        }

        // Read flags and parameters
        
        // Debuf flag: false by default
        is_debug_flag = rf.check("is_debug_flag") &&
                rf.check("is_debug_flag", Value(false)).asBool();
        yInfo() << "Flag is_debug_flat is: " << is_debug_flag;

        // Number of tones to classify: 5 by default
        int default_number_tones_output_neurons = 5;
        number_tones_output_neurons = rf.check("number_tones_output_neurons",
                                    Value(default_number_tones_output_neurons)).asInt();
        yInfo() << "Setting number_tones_output_neurons parameter to: " << number_tones_output_neurons;

        // Number of positions to classify: 9 by default
        int default_number_sound_source_neurons = 9;
        number_sound_source_neurons = rf.check("number_sound_source_neurons",
                                    Value(default_number_sound_source_neurons)).asInt();
        yInfo() << "Setting number_sound_source_neurons parameter to: " << number_sound_source_neurons;

        // Period value: 1 sec by default
        double default_update_period = 1.0;
        update_period = rf.check("update_period",
                                    Value(default_update_period)).asDouble();
        yInfo() << "Setting update_period parameter to: " << update_period;

        // Do any other set-up required here

        // Start the asynchronous and synchronous threads
        yInfo() << "Starting the thread...";
        return Thread::start();
    }

    virtual double getPeriod()
    {
        // Period of synchrnous thread (in seconds)
        return update_period;
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
        yInfo() << "Closing output tones port...";
        output_port_tones.close();
        yInfo() << "Closing output soundsource port...";
        output_port_soundsource.close();
        yInfo() << "Module has been closed!";
    }

    // Synchronous thread
    virtual bool updateModule()
    {

        // Add any synchronous operations here, visualisation, debug out prints

        return Thread::isRunning();
    }

    // Synchronous thread run forever
    void run()
    {
        // YARP timestamp
        Stamp yarpstamp;

        // Output queue for pure tones: type AE
        deque<AE> out_queue_tones;
        // Output queue for sound source: type AE
        deque<AE> out_queue_soundsource;
        // Output event: type AE
        AE out_event;

        // Raw address received from the input port
        int address = 0;

        // Forever...
        while(true) {

            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            // Do asynchronous processing here
            for(auto &qi : *q) {

                // Get the real AER address
                address = qi._coded_data;

                // If debug flag enabled, print it out
                if (is_debug_flag == true) {
                    yDebug() << "Event received and decoded address: " << address;
                }

                // Copy the address to the raw int32
                out_event._coded_data = address;
                // Copy the timestamp --> Is it necessary to pre-process?!!!!!!
                out_event.stamp = qi.stamp;

                // Add the event to its output queue
                // ----------------------------------------------------------------------------
                // WARNING!! First, we send the tones output events, and then the sound source
                // classification output events
                // ----------------------------------------------------------------------------
                if ((address >= 0) && (address < number_tones_output_neurons)) {
                    out_queue_tones.push_back(out_event);
                } else if ((address >= number_tones_output_neurons) && (address < (number_tones_output_neurons + number_sound_source_neurons))) {
                    out_queue_soundsource.push_back(out_event);
                } else {
                    yWarning() << "Received address is out of the addresses range!";
                }
                
            }

            // After processing the packet output the results
            // (only if there is something to output)

            // Tones classification queue
            if(out_queue_tones.size()) {
                output_port_tones.write(out_queue_tones, yarpstamp);
                out_queue_tones.clear();
            }
            // Sound source classification queue
            if(out_queue_soundsource.size()) {
                output_port_soundsource.write(out_queue_soundsource, yarpstamp);
                out_queue_soundsource.clear();
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
    rf.setDefaultConfigFile( "vSpinnakerEventsMapper.ini" );
    rf.configure( argc, argv );

    // Create the module
    vSpinnakerEventsMapper instance;
    return instance.runModule(rf);
}
