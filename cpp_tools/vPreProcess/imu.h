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
#include "raw_datatype.h"

class imuFunctions 
{
private:
    ev::BufferedPort<ev::IMUS> output;
    ev::packet<ev::IMUS>* p{nullptr};

public:
    bool open(std::string mname)
    {
        std::string port_name = mname + "/imu/IMUS:o";
        if(!output.open(port_name)) {
            yError() << "Could not open" << port_name;
            return false;
        }
        p = &(output.prepare());
        return true;
    }
    void process(ev::IMUS *datum)
    {
        if(p == nullptr) return;
        p->push_back(*datum);

    }
    void send(yarp::os::Stamp stamp, double duration)
    {
        if(p && p->size()) {
            p->duration(duration);
            output.setEnvelope(stamp);
            output.write();
            p = &(output.prepare());
        }

    }
    void close()
    {
        p = nullptr;
        output.unprepare();
        output.close();
    }

};