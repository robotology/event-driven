/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/os/RpcClient.h>

#include <iCub/emorph/all.h>
//#include <iCub/emorph/eventCodec.h>

#include "trackerPool.h"

#include <time.h>
#include <string>


class EventBottleManager : public yarp::os::BufferedPort<emorph::vBottle>
{
    private:

        std::string 				                moduleName;         //string containing module name
        std::string 				                inPortName;        	//string containing events input port name
        std::string 				                leftPortName;	    //string containing image output port name
        std::string 				                rightPortName;	    //string containing image output port name
        std::string 				                outPortName;	    //string containing events output port name
    
        //yarp::os::BufferedPort<eventBottle>         inPort;             //input port for the eventBottles from aexGrabber or dataSetPlayer
        yarp::os::Port                              leftPort;           //output port for the image left
        yarp::os::Port                              rightPort;          //output port for the image right
        yarp::os::BufferedPort<emorph::vBottle>             outPort;            //output port for the eventBottle with the new events computed by the module

        std::string                                 fileName;

        FILE                                        *output_file;       // File for outputing the data

        // Cluster initialization values
        double                                      alphaShape;
        double                                      alphaPos;
        // Cluster (in)activation thresholds (percent of activation)
        double                                      downThr;            // percentage of activity for inactivating the tracker
        double                                      upThr;              // percentage of activity for activating the tracker
        double                                      decay_tau;
    
        int                                         min_nb_ev;          // Threshold for updating the position
        int                                         numClusters;        //number of clusters (const)
        yarp::sig::Vector                           numEventsPerCluster;
        yarp::sig::Vector                           currentEventNumbers, previousEventNumbers;

        std::vector<double>                         x_coll_left, y_coll_left, x_coll_right, y_coll_right;        // Variables that will hold the coordinates of the collisions

        int                                         numIters;
    
        bool                                        moveEyes;
        
        int                                         last_t_display;
        int                                         dt;
    
        yarp::sig::ImageOf<yarp::sig::PixelRgb> gray_image;
        yarp::sig::ImageOf<yarp::sig::PixelRgb> left_image;
        yarp::sig::ImageOf<yarp::sig::PixelRgb> right_image;

    
    public:
    
        //create trackers, left and right
        TrackerPool *tracker_pool_left;
        TrackerPool *tracker_pool_right;
        
        /**
         * constructor
         * @param moduleName is passed to the thread in order to initialise all the ports correctly (default yuvProc)
         */
        EventBottleManager( const std::string &moduleName, std::string &fileName, double &alphaShape, double &alphaPos, double &upThr, double &downThr, double &decay_tau);
        ~EventBottleManager();

        bool    open();
        bool    init();
        void    close();
        void    onRead(emorph::vBottle &bot);
        void    interrupt();
  
};

class EventClustering:public yarp::os::RFModule
{
    /* module parameters */
    std::string             moduleName;
    std::string             rpcPortName;
    yarp::os::RpcServer     rpcPort;

    std::string             fileName;
    double                  alphaShape;
    double                  alphaPos;
    double                  downThr;
    double                  upThr;
    double                  decay_tau;
    /* pointer to a new manager */
    EventBottleManager      *eventBottleManager;
    bool                    closing;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module

    double getPeriod();
    bool updateModule();

    /**
     * @brief function that sets the output file name for saving the clustered data
     */
    //void setFileName(std::string value) {fileName = value; };
 
    /**
     * @brief function that sets the alpha shape
     */
    //void setAlphaShape(int value) {alphaShape = value; };
 
    /**
     * @brief function that sets the alpha shape
     */
    //void setAlphaPos(int value) {alphaPos = value; };



};


#endif
//empty line to make gcc happy
