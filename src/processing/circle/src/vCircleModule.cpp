/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include "vCircleModule.h"
#include <opencv2/opencv.hpp>

/**********************************************************/
bool vCircleModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("vCircleFinder")
                                      ).asString();
    setName(moduleName.c_str());


    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool vCircleModule::interruptModule()
{
    eventBottleManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCircleModule::close()
{
    eventBottleManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCircleModule::updateModule()
{
    yarp::sig::Matrix act =
            eventBottleManager.circleFinder.activity.copyAllActivity();
    cv::Mat image(act.rows(), act.cols(), CV_8U);
    for(int i = 0; i < act.rows(); i++) {
        for(int j = 0; j < act.cols(); j++) {
            image.at<char>(i, j) = act(i, j);
        }
    }
    cv::imshow("Activity Debug", image);
    cv::waitKey(1);


    return true;
}

/**********************************************************/
double vCircleModule::getPeriod()
{
    return 0.1;
}


/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    
}
/**********************************************************/
bool EventBottleManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);
    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    //close ports
    this->close();
    outPort.close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    this->interrupt();
    outPort.interrupt();

}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q;
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    // get the event queue in the vBottle bot
    bot.getAll(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        circleFinder.localCircleEstimate(*v);

        //unwrap timestamp
        //unsigned long int ts = unwrapper((*qi)->getStamp());

        //process

        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);


    }
    //send on the processed events
    //outPort.write();

}

//empty line to make gcc happy
