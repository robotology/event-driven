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

#include "vFramer.h"
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
//vFramerModule
/*////////////////////////////////////////////////////////////////////////////*/


bool vFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    //admin options
    std::string moduleName =
            rf.check("name", yarp::os::Value("/vFramer")).asString();
    setName(moduleName.c_str());

    int retinaHeight = rf.check("height", yarp::os::Value(240)).asInt();
    int retinaWidth = rf.check("width", yarp::os::Value(304)).asInt();

    double eventWindow =
            rf.check("eventWindow", yarp::os::Value(0.1)).asDouble();
    eventWindow *= vtsHelper::vtsscaler;
    eventWindow = std::min(eventWindow, vtsHelper::max_stamp / 2.0);

    double isoWindow = rf.check("isoWindow", yarp::os::Value(1.0)).asDouble();
    isoWindow *= vtsHelper::vtsscaler;
    isoWindow = std::min(isoWindow, vtsHelper::max_stamp / 2.0);

    int frameRate = rf.check("frameRate", yarp::os::Value(30)).asInt();
    period = 1.0 / frameRate;

    useTimeout = rf.check("timeout") &&
            rf.check("timeout", yarp::os::Value(true)).asBool();

    bool flip = rf.check("flip") &&
            rf.check("flip", yarp::os::Value(true)).asBool();

    bool forceRender = rf.check("forcerender") &&
            rf.check("forcerender", yarp::os::Value(true)).asBool();
    if(forceRender) {
        vReader.setStrictUpdatePeriod(vtsHelper::vtsscaler * period);
        period = 0;
    }

    //viewer options
    //set up the default channel list
    yarp::os::Bottle tempDisplayList, *bp;
    tempDisplayList.addInt(0);
    tempDisplayList.addString("/Left");
    bp = &(tempDisplayList.addList()); bp->addString("AE");
    tempDisplayList.addInt(1);
    tempDisplayList.addString("/Right");
    bp = &(tempDisplayList.addList()); bp->addString("AE");

    //set the output channels
    yarp::os::Bottle * displayList = rf.find("displays").asList();
    if(!displayList)
        displayList = &tempDisplayList;

    yInfo() << displayList->toString();

    if(displayList->size() % 3) {
        std::cerr << "Error: display incorrectly configured in provided "
                     "settings file." << std::endl;
        return false;
    }

    int nDisplays = displayList->size() / 3;

    //for each channel open a vFrame and an output port
    channels.resize(nDisplays);
    outports.resize(nDisplays);
    drawers.resize(nDisplays);
    q_snaps.resize(nDisplays);

    for(int i = 0; i < nDisplays; i++) {

        //extract the channel integer value
        channels[i] = displayList->get(i*3).asInt();

        //extract the output port name and open it
        std::string outportname = displayList->get(i*3 + 1).asString();
        outports[i] = new yarp::os::BufferedPort<
                yarp::sig::ImageOf<yarp::sig::PixelBgr> >;
        if(!outports[i]->open(moduleName + outportname))
            return false;

        //create the draw types
        yarp::os::Bottle * drawtypelist = displayList->get(i*3 + 2).asList();
        for(unsigned j = 0; j < drawtypelist->size(); j++) {
            vDraw * newDrawer = createDrawer(drawtypelist->get(j).asString());
            if(newDrawer) {
                newDrawer->setRetinaLimits(retinaWidth, retinaHeight);
                newDrawer->setTemporalLimits(eventWindow, 2.0*vtsHelper::vtsscaler);
                newDrawer->setFlip(flip);
                newDrawer->initialise();
                drawers[i].push_back(newDrawer);
                if(!vReader.open(moduleName, newDrawer->getEventType()))
                    yError() << "Could not open input port";
            } else {
                yError() << "Could not find draw tag "
                          << drawtypelist->get(j).asString()
                          << ". No drawer created";
            }
        }

        //preallocate the queue snapshot memory
        q_snaps[i].resize(drawtypelist->size());
    }

    return true;

}

bool vFramerModule::interruptModule()
{
    vReader.close();
    return true;
}

bool vFramerModule::close()
{
    vReader.close();
    for(unsigned int i = 0; i < outports.size(); i++)
        outports[i]->close();
    return true;
}

void vFramerModule::sendBlanks() {

    for(unsigned int i = 0; i < channels.size(); i++) {

        //get the image to be written and make a cv::Mat pointing to the same
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &o = outports[i]->prepare();
        cv::Mat canvas = cv::cvarrToMat((IplImage *)o.getIplImage());
        drawers[i][0]->resetImage(canvas);

        //tell the actual YARP image what size the final image became
        o.resize(canvas.cols, canvas.rows);

        //send through the envelope and write
        yarp::os::Stamp cEnv = vReader.getystamp();
        if(cEnv.isValid()) outports[i]->setEnvelope(cEnv);
        outports[i]->write();
    }

}

bool vFramerModule::updateModule()
{

    //check if an update is needed, send blank images after a timeout
    static double pTime = 0;
    if(!vReader.hasUpdated()) {
        if(useTimeout && yarp::os::Time::now() - pTime > 2.0) {
            sendBlanks();
            pTime = yarp::os::Time::now();
        }
        return true;
    }
    pTime = yarp::os::Time::now();

    bool use_synchronisation = false;
    //snapshot the events
    //double dt1 = Time::now();
    if(use_synchronisation) {
    for(unsigned int i = 0; i < channels.size(); i++) {
        for(unsigned int j = 0; j < drawers[i].size(); j++) {
            q_snaps[i][j] = vReader.queryWindow(drawers[i][j]->getEventType(),
                                                channels[i]);
        }
    }
    }

    //check if we have unprocessed events
    static int puqs = 0;
    int uqs = vReader.queryMaxUnproced();
    if(uqs || puqs) {
        //yInfo() << uqs << "unprocessed queues";
        if(uqs)
            yInfo() << vReader.delayStats();
        puqs = uqs;
    }

    //std::cout << "Snappshotting: " << Time::now() - dt1 << std::endl;



    //snapshot the current time
    vReader.updateStamps();
    int current_vts = vReader.getvstamp();
    yarp::os::Stamp cEnv = vReader.getystamp();


    //trim eSet based on the synchronisation time
//    if(use_synchronisation) {
//        for(unsigned int i = 0; i < channels.size(); i++) {
//            for(unsigned int j = 0; j < drawers[i].size(); j++) {
//                while(q_snaps[i][j].size() &&
//                      q_snaps[i][j].back()->stamp > current_vts)
//                    q_snaps[i][j].pop_back();

//            }
//        }
//    }

    //for each output image needed
    double acct2 = 0;
    for(unsigned int i = 0; i < channels.size(); i++) {

        double dt2 = Time::now();
        //get the image to be written and make a cv::Mat pointing to the same
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &o = outports[i]->prepare();
        cv::Mat canvas = cv::cvarrToMat((IplImage *)o.getIplImage());

        //the first drawer will reset the base image, then drawing proceeds
        drawers[i][0]->resetImage(canvas);


        if(use_synchronisation) {
            //we can synchronise all drawers
            for(unsigned int j = 0; j < drawers[i].size(); j++)
                drawers[i][j]->draw(canvas, q_snaps[i][j], current_vts);

        } else {
            //we can't synch, just each streams current time individually
            for(unsigned int j = 0; j < drawers[i].size(); j++) {
                //if(q_snaps[i][j].empty()) continue;
                drawers[i][j]->draw(canvas, vReader.queryWindow(drawers[i][j]->getEventType(),
                                                                channels[i]),
                                    -1);
            }
        }
        acct2 += Time::now() - dt2;


        //tell the actual YARP image what size the final image became
        o.resize(canvas.cols, canvas.rows);

        //write
        if(cEnv.isValid()) outports[i]->setEnvelope(cEnv);
        outports[i]->write();
    }

    //std::cout << "Drawing: " << 1.0 / acct2 << std::endl;

    return true;
}

double vFramerModule::getPeriod()
{
    return period;
}

vFramerModule::~vFramerModule()
{
    for(unsigned int i = 0; i < drawers.size(); i++) {
        for(unsigned int j = 0; j < drawers[i].size(); j++) {
            delete drawers[i][j];
        }
    }

    for(unsigned int i = 0; i < outports.size(); i++) {
        delete outports[i];
    }
}

