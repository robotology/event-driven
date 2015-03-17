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
//    cv::Mat image(128, 128, CV_8U); image.setTo(0);

////    yarp::sig::Matrix act =
////            eventBottleManager.circleFinder.activity.copyAllActivity();
////    for(int i = 0; i < act.rows(); i++) {
////        for(int j = 0; j < act.cols(); j++) {
////            image.at<char>(i, j) = act(i, j);
////        }
////    }

//    emorph::vQueue q; emorph::vQueue::iterator qi;
//    eventBottleManager.tempBottle.getAll(q);
//    for(qi = q.begin(); qi != q.end(); qi++) {
//        emorph::ClusterEvent *cv = (*qi)->getAs<emorph::ClusterEvent>();
//        if(!cv) continue;
//        cv::circle(image, cv::Point2i(cv->getXCog(), cv->getYCog()), 4, CV_RGB(255, 255, 255), CV_FILLED);
//    }

//    cv::imshow("Activity Debug", image);
//    cv::waitKey(1);


    return true;
}

/**********************************************************/
double vCircleModule::getPeriod()
{
    return 0.3;

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
    //outBottle.clear();
    outBottle = bot;

    //cv::Mat image(128*4, 128*4, CV_8U); image.setTo(0);
    emorph::activityMat estimate(128, 128, 1000000000, 5, 8);
    // get the event queue in the vBottle bot
    bot.getAll(q);

    tempBottle.clear();
    for(qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        if(v->getChannel()) continue;
        emorph::ClusterEvent * cv = circleFinder.localCircleEstimate(*v);
        if(!cv) continue;
        emorph::AddressEvent tevent(*v);
        tevent.setStamp(v->getStamp());
        tevent.setX(cv->getXCog());
        tevent.setY(cv->getYCog());

        //outBottle.addEvent(*cv);
        //tempBottle.addEvent(*cv);
        //cv::circle(image, cv::Point2i(cv->getYCog()*4, (128 - cv->getXCog())*4), 4, CV_RGB(255, 255, 255), CV_FILLED);
        estimate.addEvent(tevent);
    }

    double mact = 0;
    cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
    for(int x = 0; x < 128; x++) {
        for(int y = 0; y < 128; y++) {
            double a = estimate.queryActivity(x, y);
            mact = std::max(a, mact);
            image2.at<float>(127 - x, y) = a;
        }
    }
    image2 = image2 * (1/mact);
    cv::Mat image(512, 512, CV_32F);
    cv::resize(image2, image, image.size());

    cv::imshow("Local Estimate", image);
    cv::waitKey(1);




    //send on the processed events
    outPort.write();
    //std::cout << outBottle.toString() << std::endl;

}

//empty line to make gcc happy
