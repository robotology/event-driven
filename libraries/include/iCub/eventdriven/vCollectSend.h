/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __VCOLLECTSEND__
#define __VCOLLECTSEND__

#include <iCub/eventdriven/vCodec.h>
#include <iCub/eventdriven/vBottle.h>
#include <yarp/os/all.h>

namespace ev {

/// \brief an output port that can safely accept events from multiple threads
/// and sends them at a fixed output rate.
class collectorPort : public yarp::os::RateThread
{
private:

    vBottle filler;
    yarp::os::BufferedPort<vBottle> sendPort;
    yarp::os::Mutex m;
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
        filler.addEvent(v);
        ystamp = y;
        m.unlock();

    }

    /// \brief on each call of the thread, all events that have been added are
    /// sent on the port in a vBottle. If no events have been added, a vBottle
    /// is not sent.
    void run() {

        if(filler.size()) {
            vBottle &b = sendPort.prepare();

            m.lock();
            b = filler;
            filler.clear();
            sendPort.setEnvelope(ystamp);
            m.unlock();

            sendPort.write();

        }
    }

};



}

#endif
