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
    collectorPort() : RateThread(1.0) {}

    bool open(std::string name) {

        return sendPort.open(name);

    }

    void pushevent(event<> v, yarp::os::Stamp y) {

        m.lock();
        filler.addEvent(v);
        ystamp = y;
        m.unlock();

    }

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
