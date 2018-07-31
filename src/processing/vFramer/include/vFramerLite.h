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

// \defgroup Visualisation Visualisation
// \defgroup vFramer vFramer
// \ingroup Visualisation
// \brief converts the event-stream to an yarpview-able image

#ifndef __vFramerLite__
#define __vFramerLite__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/eventdriven/all.h>
#include <opencv2/opencv.hpp>
#include <map>

#include "vDraw.h"

using namespace ev;
using namespace yarp::os;
using yarp::sig::ImageOf;
using yarp::sig::PixelBgr;
using std::vector;
using std::deque;
using std::string;
using std::map;

class channelInstance : public RateThread {

private:

    string channel_name;
    unsigned int limit_time;

    map<string, vGenReadPort> read_ports;
    map<string, vQueue> event_qs;
    vector<vDraw *> drawers;
    BufferedPort< ImageOf<PixelBgr> > image_port;

    bool updateQs();

    //events are removed in batches corresponding to packets to reduce
    //the amount of timestamp comparisons required.
    map<string, unsigned int> total_time;
    map<string, deque<unsigned int> > bookmark_time;
    map<string, deque<unsigned int> > bookmark_n_events;
    map<string, int> prev_vstamp;

public:

    channelInstance(string channel_name);
    bool addDrawer(string drawer_name, unsigned int width,
                   unsigned int height, unsigned int window_size, bool flip);

    bool threadInit();
    void run();
    void threadRelease();

    string getName();

};


/**
 * @brief The vFramerModule class runs the event reading and channel splitting,
 * the drawing modules, and the yarp image output buffers. Images are created
 * at a rate equal to the rate of this thread.
 */
class vFramerModule : public yarp::os::RFModule {

private:

    vector<channelInstance *> publishers;

public:

    virtual ~vFramerModule();

    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the modulereturn

    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    virtual bool updateModule();
    virtual double getPeriod();
};


#endif //vFramer

