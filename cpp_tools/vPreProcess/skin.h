/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once
#include <string>
#include <yarp/os/Log.h>


class skinFunctions 
{
private:

    ev::BufferedPort<ev::skinAE> output_events;
    ev::packet<ev::skinAE>* p_events{nullptr};
    ev::BufferedPort<ev::skinSample> output_samples;
    ev::packet<ev::skinSample>* p_samples{nullptr};

    bool open_events(std::string mname)
    {
        std::string port_name = mname + "/skin/SKE:o";
        if(!output_events.open(port_name)) {
            yError() << "Could not open" << port_name;
            return false;
        }
        p_events = &(output_events.prepare());
        return true;
    }

    bool open_samples(std::string mname)
    {
        std::string port_name = mname + "/skin/SKS:o";
        if(!output_samples.open(port_name)) {
            yError() << "Could not open" << port_name;
            return false;
        }
        p_samples = &(output_samples.prepare());
        return true;
    }

public:

    bool open(std::string mname)
    {
        if(!open_events(mname))
            return false;
        if(!open_samples(mname))
            return false;
        return true;
    }

    void process(ev::encoded *datum)
    {
        static ev::skinSample ss;
        static bool expect_skin_value{false};

        if(IS_SKSAMPLE(datum->data)) {
            if(p_samples) {
                if(IS_SSA(datum->data)) { //this is sent first
                    ss.address = *(ev::skinAE *)datum;
                    if(expect_skin_value) yError() << "mismatch skin samples";
                    expect_skin_value = true;
                } else { //IS_SSV -> this is sent second
                    ss.value = *(ev::skinValue *)datum;
                    if(expect_skin_value) p_samples->push_back(ss);
                    else yError() << "mismatch skin samples";
                    expect_skin_value = false;
                }
            }
        } else {
            if(p_events) p_events->push_back(*(ev::skinAE *)datum);
        }

    }
    void send(yarp::os::Stamp stamp, double duration)
    {
        if(p_events && p_events->size()) {
            p_events->duration(duration);
            output_events.setEnvelope(stamp);
            output_events.write();
            p_events = &(output_events.prepare());
        }

        if (p_samples && p_samples->size()) {
            p_samples->duration(duration);
            output_samples.setEnvelope(stamp);
            output_samples.write();
            p_samples = &(output_samples.prepare());
        }
    }
    void close()
    {
        p_events = nullptr;
        output_events.unprepare();
        output_events.close();

        p_samples = nullptr;
        output_samples.unprepare();
        output_samples.close();
    }

};