/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
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

#ifndef __VCOLLECTSEND__
#define __VCOLLECTSEND__

#include "event-driven/vCodec.h"
#include "event-driven/vPort.h"
#include <yarp/os/all.h>

namespace ev {

/// \brief an output port that can safely accept events from multiple threads
/// and sends them at a fixed output rate.
class collectorPort : public yarp::os::RateThread
{
private:

    vQueue filler;
    ev::vWritePort sendPort;
    std::mutex m;
    yarp::os::Stamp ystamp;



public:

    /// \brief constructor
    collectorPort() : RateThread(1.0) {}

    /// \brief open the output port
    bool open(std::string name) {

        return sendPort.open(name);

    }

    /// \brief add an event to be sent on next thread execution
    void pushevent(event<> v, yarp::os::Stamp y) {

        m.lock();
        filler.push_back(v);
        ystamp = y;
        m.unlock();

    }

    /// \brief on each call of the thread, all events that have been added are
    /// sent on the port in a vBottle. If no events have been added, a vBottle
    /// is not sent.
    void run() {

        if(filler.size()) {

            m.lock();
            sendPort.write(filler, ystamp);
            filler.clear();
            m.unlock();

        }
    }

};



}

#endif
