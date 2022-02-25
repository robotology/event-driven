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

class audioFunctions 
{
private:
    ev::BufferedPort<ev::earEvent> output;
    ev::packet<ev::earEvent>* p{nullptr};

public:
    bool open(std::string mname)
    {
        std::string port_name = mname + "/cochlea/EAR:o";
        if(!output.open(port_name)) {
            yError() << "Could not open" << port_name;
            return false;
        }
        p = &(output.prepare());
        return true;
    }
    void process(ev::earEvent *datum)
    {
        if(p == nullptr) return;
        p->push_back(*datum);

    }
    void send(yarp::os::Stamp stamp, double duration)
    {
        if(p && p->size()) {
            p->duration(duration);
            p->envelope() = stamp;
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