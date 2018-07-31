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

#include "vFramerLite.h"
#include <sstream>

using namespace ev;

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find yarp network";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vFramer.ini" );
    rf.configure( argc, argv );

    vFramerModule framerModule;
    return framerModule.runModule(rf);
}

/*////////////////////////////////////////////////////////////////////////////*/
//channelInstance
/*////////////////////////////////////////////////////////////////////////////*/
channelInstance::channelInstance(string channel_name) : RateThread(0.1)
{
    this->channel_name = channel_name;
    this->limit_time = 1.0 * vtsHelper::vtsscaler;
}

string channelInstance::getName()
{
    return channel_name;
}

bool channelInstance::addDrawer(string drawer_name, unsigned int width,
                                unsigned int height, unsigned int window_size,
                                bool flip)
{
    //make the drawer
    vDraw * new_drawer = createDrawer(drawer_name);
    if(new_drawer) {
        new_drawer->setRetinaLimits(width, height);
        new_drawer->setTemporalLimits(window_size, limit_time);
        new_drawer->setFlip(flip);
        new_drawer->initialise();
        drawers.push_back(new_drawer);
    } else {
        return false;
    }

    string event_type = new_drawer->getEventType();

    //check to see if we need to open a new input port
    if(read_ports.count(event_type))
        return true;

    //open the port
    total_time[event_type] = 0;
    prev_vstamp[event_type] = 0;
    return read_ports[event_type].open(channel_name + "/" + event_type + ":i");

}

bool channelInstance::threadInit()
{
    return image_port.open(channel_name + "/image:o");
}

bool channelInstance::updateQs()
{
    bool updated = false;
    Stamp yarp_stamp;
    //fill up the q's as much as possible
    map<string, int> qs_available;
    std::map<string, vGenReadPort>::iterator port_i;
    for(port_i = read_ports.begin(); port_i != read_ports.end(); port_i++) {
        qs_available[port_i->first] = port_i->second.queryunprocessed();
        if(qs_available[port_i->first]) updated = true;
    }

    for(port_i = read_ports.begin(); port_i != read_ports.end(); port_i++) {
        const string &event_type = port_i->first;
        for(int i = 0; i < qs_available[event_type]; i++) {
            const vQueue *q = port_i->second.read(yarp_stamp);

            int q_dt = (int)q->back()->stamp - prev_vstamp[event_type];
            if(q_dt < 0) q_dt += vtsHelper::max_stamp;

            prev_vstamp[event_type] = (int)q->back()->stamp;
            total_time[event_type] += q_dt;
            bookmark_time[event_type].push_back(q_dt);
            bookmark_n_events[event_type].push_back(q->size());

            for(unsigned j = 0; j < q->size(); j++)
                event_qs[event_type].push_back(q->at(j));
        }
        while(total_time[event_type] > limit_time) {
            for(unsigned int i = 0; i < bookmark_n_events[event_type].front(); i++)
                event_qs[event_type].pop_front();
            total_time[event_type] -= bookmark_time[event_type].front();
            bookmark_time[event_type].pop_front();
            bookmark_n_events[event_type].pop_front();
        }
    }

    return updated;
}


void channelInstance::run()
{

    if(!updateQs())
        return;


    //get the image to be written and make a cv::Mat pointing to the same
    ImageOf<PixelBgr> &o = image_port.prepare();
    cv::Mat canvas = cv::cvarrToMat((IplImage *)o.getIplImage());

    //the first drawer will reset the base image, then drawing proceeds
    drawers.front()->resetImage(canvas);

    vector<vDraw *>::iterator drawer_i;
    for(drawer_i = drawers.begin(); drawer_i != drawers.end(); drawer_i++) {
        (*drawer_i)->draw(canvas, event_qs[(*drawer_i)->getEventType()], -1);
    }


    //tell the actual YARP image what size the final image became
    o.resize(canvas.cols, canvas.rows);

    //write
    //if(cEnv.isValid()) outports[i]->setEnvelope(cEnv);
    image_port.write();

    //updateQs();

}

void channelInstance::threadRelease()
{
    //close input ports
    std::map<string, vGenReadPort>::iterator port_i;
    for(port_i = read_ports.begin(); port_i != read_ports.end(); port_i++) {
        port_i->second.close();
    }

    //close output port
    image_port.close();

    //delete allocated memory
    std::vector<vDraw *>::iterator drawer_i;
    for(drawer_i = drawers.begin(); drawer_i != drawers.end(); drawer_i++) {
        delete *drawer_i;
    }

}


/*////////////////////////////////////////////////////////////////////////////*/
//channelInstance
/*////////////////////////////////////////////////////////////////////////////*/
bool vFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    //admin options
    string moduleName = rf.check("name", Value("/vFramer")).asString();
    setName(moduleName.c_str());

    int height = rf.check("height", Value(240)).asInt();
    int width = rf.check("width", Value(304)).asInt();

    double eventWindow = rf.check("eventWindow", Value(0.1)).asDouble();
    eventWindow *= vtsHelper::vtsscaler;
    eventWindow = std::min(eventWindow, vtsHelper::max_stamp / 2.0);

    double isoWindow = rf.check("isoWindow", Value(1.0)).asDouble();
    isoWindow *= vtsHelper::vtsscaler;
    isoWindow = std::min(isoWindow, vtsHelper::max_stamp / 2.0);

    int frameRate = rf.check("frameRate", Value(30)).asInt();
    double period = 1000.0 / frameRate;

    //bool useTimeout =
    //        rf.check("timeout") && rf.check("timeout", Value(true)).asBool();
    bool flip =
            rf.check("flip") && rf.check("flip", Value(true)).asBool();
    //bool forceRender =
    //        rf.check("forcerender") &&
    //        rf.check("forcerender", Value(true)).asBool();
//    if(forceRender) {
//        vReader.setStrictUpdatePeriod(vtsHelper::vtsscaler * period);
//        period = 0;
//    }

    //viewer options
    //set up the default channel list
    yarp::os::Bottle tempDisplayList, *bp;
    tempDisplayList.addString("/Left");
    bp = &(tempDisplayList.addList()); bp->addString("AE");
    tempDisplayList.addString("/Right");
    bp = &(tempDisplayList.addList()); bp->addString("AE");

    //set the output channels
    yarp::os::Bottle * displayList = rf.find("displays").asList();
    if(!displayList)
        displayList = &tempDisplayList;

    yInfo() << displayList->toString();

    if(displayList->size() % 2) {
        std::cerr << "Error: display incorrectly configured in provided "
                     "settings file." << std::endl;
        return false;
    }

    int nDisplays = displayList->size() / 2;


    for(int i = 0; i < nDisplays; i++) {

        string channel_name =
                moduleName + displayList->get(i*2).asString();

        channelInstance * new_ci = new channelInstance(channel_name);
        new_ci->setRate(period);

        Bottle * drawtypelist = displayList->get(i*2 + 1).asList();
        for(unsigned int j = 0; j < drawtypelist->size(); j++)
        {
            string draw_type = drawtypelist->get(j).asString();
            if(!new_ci->addDrawer(draw_type, width, height, eventWindow, flip))
            {
                yError() << "Could not create specified publisher"
                         << channel_name << draw_type;
                return false;
            }
        }

        publishers.push_back(new_ci);

    }

    vector<channelInstance *>::iterator pub_i;
    for(pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++) {
        if(!(*pub_i)->start()) {
            yError() << "Could not start publisher" << (*pub_i)->getName();
            return false;
        }
    }

    return true;
}

bool vFramerModule::interruptModule()
{
    vector<channelInstance *>::iterator pub_i;
    for(pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
        (*pub_i)->stop();

    return true;
}

bool vFramerModule::close()
{
    vector<channelInstance *>::iterator pub_i;
    for(pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
        (*pub_i)->stop();

    return true;
}

bool vFramerModule::updateModule()
{
    return !isStopping();
}

double vFramerModule::getPeriod()
{
    return 1.0;
}

vFramerModule::~vFramerModule()
{

}

