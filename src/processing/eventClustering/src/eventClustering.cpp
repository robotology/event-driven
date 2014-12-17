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

#include "eventClustering.h"
#include "trackerPool.h"
#include "blobTracker.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace emorph::ecodec;

/**********************************************************/
bool EventClustering::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("eventClustering"), "module name (string)").asString();

    setName(moduleName.c_str());

    rpcPortName  =  "/";
    rpcPortName +=  getName();
    rpcPortName +=  "/rpc:i";

    if (!rpcPort.open(rpcPortName.c_str()))
    {
        fprintf(stdout, "%s : Unable to open port %s\n", getName().c_str(), rpcPortName.c_str());
        return false;
    }

    attach(rpcPort);


    /*
    * set the file name for saving cluster data
    */
    fileName = rf.check("filename", 
                        Value("clusters.txt"), 
                        "file name (string)").asString();
    fprintf(stdout,"output file %s \n", fileName.c_str());
    //eventBottleManager ->setFileName(fileName);

    /*
    * set the alpha shape of the gauss cluster
    */
    alphaShape = rf.check("alpha_shape", 
                        Value(0.01),
                        "alpha shape (double)").asDouble();
    fprintf(stdout,"alpha shape %f \n", alphaShape);
    //eventBottleManager ->setAlphaShape(alphaShape);

    /*
    * set the alpha pos of the gauss cluster
    */
    alphaPos = rf.check("alpha_pos", 
                        Value(0.1),
                        "alpha pos (double)").asDouble();
    fprintf(stdout,"alpha pos %f \n", alphaPos);
    //eventBottleManager ->setAlphaPos(alphaPos);

    /*
     * set the activation threshold of the gauss cluster
     */
    upThr = rf.check("up_thr",
                     Value(5),
                     "up thr (double)").asDouble();
    fprintf(stdout,"up thr %f \n", upThr);

    /*
     * set the inactivation threshold of the gauss cluster
     */
    downThr = rf.check("down_thr",
                     Value(1),
                     "down thr (double)").asDouble();
    fprintf(stdout,"down thr %f \n", downThr);

    closing = false;
    
    /* create the thread and pass pointers to the module parameters */
    eventBottleManager = new EventBottleManager( moduleName, fileName, alphaShape, alphaPos, upThr, downThr);

    /* initialize variables */
    //eventBottleManager->init();
    if(eventBottleManager->init())
    {
        fprintf(stdout,"eventBottleManager init\n");
    }


   /* now open the manager to do the work */
    if(eventBottleManager->open())
    {
        fprintf(stdout,"eventBottleManager open\n");
    }
    return true ;
}

/**********************************************************/
bool EventClustering::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager->interrupt();
    return true;
}

/**********************************************************/
bool EventClustering::close()
{
    rpcPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");
    closing = true;
    eventBottleManager->close();
    fprintf(stdout, "deleting thread\n");
    delete eventBottleManager;
    fprintf(stdout, "done deleting thread\n");
    return true;
}

/**********************************************************/
bool EventClustering::updateModule()
{
    return !closing;
}

/**********************************************************/
double EventClustering::getPeriod()
{
    return 0.1;
}

/**********************************************************/
EventBottleManager::~EventBottleManager()
{

}

/**********************************************************/
EventBottleManager::EventBottleManager( const string &moduleName, std::string &fileName, double &alphaShape, double &alphaPos, double &upThr, double &downThr )
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
    this->fileName = fileName;
    this->alphaPos = alphaPos;
    this->alphaShape = alphaShape;
    this->downThr = downThr;
    this->upThr = upThr;
    
}
/**********************************************************/
bool EventBottleManager::init()
{

    // Initial parameters of the Tracker Pool
    double  sig_x = 5;
    double  sig_y = 5;
    double  sig_xy = 0;
    double  k = 2;
    double  max_dist = 30;
    bool    fixed_shape = false;
    double  tau_act = 10000;
    double  delete_thresh = 0.00000001;
    double  alpha_rep = 2;
    int     d_rep = 40;
    int     max_nb_trackers = 40;
    int     nb_ev_reg = 50;
    double  dist_thresh = 30;
    double  vel_thresh = 50;
    double  acc_thresh = 300;
    int     s;
    
    output_file = fopen (fileName.c_str(), "w");
    if (output_file == NULL) 
        perror ("Error opening file");
  
    // Create the trackers
    tracker_pool_left = new TrackerPool(sig_x, sig_y, sig_xy, alphaPos, alphaShape, k, max_dist, fixed_shape, tau_act, upThr, downThr, delete_thresh, alpha_rep, d_rep, max_nb_trackers, nb_ev_reg);
    tracker_pool_left->set_collision_det_param(dist_thresh, vel_thresh, acc_thresh);
    tracker_pool_left->get_pool_size(s);
    fprintf(stdout, "cluster pool left size s =%d\n",s);

    tracker_pool_right = new TrackerPool(sig_x, sig_y, sig_xy, alphaPos, alphaShape, k, max_dist, fixed_shape, tau_act, upThr, downThr, delete_thresh, alpha_rep, d_rep, max_nb_trackers, nb_ev_reg);
    tracker_pool_right->set_collision_det_param(dist_thresh, vel_thresh, acc_thresh);
    tracker_pool_right->get_pool_size(s);

    fprintf(stdout, "cluster pool right size s =%d\n",s);

    // Initialization of the images - correct size (128*128 from DVS sensor)

    // Output
    gray_image.resize(128,128);
    gray_image.zero();
    
    for(int ii=0; ii<128; ii++)
    {
        for(int jj=0; jj<128; jj++)
        {
            gray_image.pixel(ii, jj) = yarp::sig::PixelRgb(127, 127, 127);
        }  
    }  

    // Left
    left_image.resize(128,128);
    left_image.zero();

    // Right
    right_image.resize(128,128);
    right_image.zero();

    numEventsPerCluster.resize(5);
    currentEventNumbers.resize(128,128);
    
    // Threshold for updating the position
    min_nb_ev = 1;
    
    // Define number of clusters
    numClusters = 5;
    numIters = 0;
    moveEyes = false;
    
    last_t_display = 0;
    dt = 10000;

    return true;
}

/**********************************************************/
bool EventBottleManager::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + "/eventBottle:i";
    BufferedPort<eventBottle >::open(inPortName.c_str());

    leftPortName = "/" + moduleName + "/leftImage:o";
    leftPort.open( leftPortName.c_str() );

    rightPortName = "/" + moduleName + "/rightImage:o";
    rightPort.open( rightPortName.c_str() );

    outPortName = "/" + moduleName + "/eventBottle:o";
    outPort.open( outPortName.c_str() );

    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    fprintf(stdout,"now closing ports...\n");

    BufferedPort<eventBottle >::close();
    leftPort.close();
    rightPort.close();
    outPort.close();
    
    delete tracker_pool_left;
    delete tracker_pool_right;
    
    fprintf(stdout,"finished closing the read port...\n");
    fclose(output_file);

}

/**********************************************************/
void EventBottleManager::interrupt()
{
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    BufferedPort<eventBottle >::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
}

/**********************************************************/
void EventBottleManager::onRead(eventBottle &bot)
{
    //create event queue
    eEventQueue q; 
    unsigned long ev_t;
    short ev_x;
    short ev_y;
    short pol;
    short channel;

    //decode packet
    Bottle event; // for eventBottle port eventPort

    eventBottle &evtCluster = outPort.prepare();
    
    if(eEvent::decode(*bot.get_packet(),q)) 
    {   
        int size = q.size();
        if (size > min_nb_ev)
        { //scan the queue of events
            for (int e = 0; e < size; e++)
            {
                if(q[e] != 0)
                {
                    if(q[e]->getType()=="TS") //identify the type of the packet (Time Stamp)
                    {
                        TimeStamp* ptr=dynamic_cast<TimeStamp*>(q[e]);
                        ev_t = (unsigned long) ptr->getStamp();
                        //fprintf(stdout, "Size %d TimeStamp: %d", size, (unsigned int) ptr->getStamp());
                    }
                    else if (q[e]->getType()=="AE") //identify the type of the packet (Address Event)
                    {
                        AddressEvent* ptr=dynamic_cast<AddressEvent*>(q[e]); //create the Address Event for the type
                        
                        
                        if(ptr->isValid())
                        {   // get data from AE
                            ev_x = ptr->getX();
                            ev_y = ptr->getY();
                            pol = ptr->getPolarity();
                            channel = ptr->getChannel();
                            if(ev_x>=0 && ev_x <= 127 && ev_y>=0 && ev_y <= 127)
                            {
                                if (channel == 0)
                                {
//                                    tracker_pool_left->update(ev_x, ev_y, ev_t);
                                    tracker_pool_left->update(ptr, ev_t);
                                    left_image.pixel(ev_y, ev_x) = yarp::sig::PixelRgb(255*pol, 255*pol, 255*pol);
                                    fprintf(stdout,"x: %d, trackId: %d\n",ev_x, ptr->getX());
                                }
                                else
                                {
                                    tracker_pool_right->update(ev_x, ev_y, ev_t);
                                    right_image.pixel(ev_y, ev_x) = yarp::sig::PixelRgb(255*pol, 255*pol, 255*pol);
                                }
                            }


                            if(ev_t - last_t_display >= dt || ev_t - last_t_display <= 0)
                            {
                                // We update the images of the ellipses
                                tracker_pool_left->display(left_image);
                                tracker_pool_right->display(right_image);
                                
                                // Write the images into the yarp port
                                leftPort.write(left_image);
                                rightPort.write(right_image);
                                // And reset them to gray
                                left_image = gray_image;
                                right_image = gray_image;
                                last_t_display = ev_t;
                            }
                        }
                    }
                }
            }
        }

        //Writing active tracker data to output port
        
        Bottle clusterEvents;
        int sizePool;
        std::vector<double> c_x, c_y, v_x, v_y, s_x2, s_y2, s_xy, mass, id;
        std::vector<bool> act;
        
        // left
        tracker_pool_left->get_pool_size(sizePool);
        tracker_pool_left->getTrackers(c_x, c_y, v_x, v_y, s_x2, s_y2, s_xy, mass, act, id);
        
        for (int iw=0; iw < sizePool; iw++)
        {
            int writeChannelNum = 0;

            if (mass[iw])
            {
                ClusterEventGauss eventToWrite;
                
                eventToWrite.setXCog(c_x[iw]);
                eventToWrite.setYCog(c_y[iw]);
                eventToWrite.setChannel(writeChannelNum);
                eventToWrite.setId(id[iw]);
                eventToWrite.setNumAE(mass[iw]);
                eventToWrite.setXSigma2(s_x2[iw]);
                eventToWrite.setYSigma2(s_y2[iw]);
                eventToWrite.setXYSigma(s_xy[iw]);
                //eventToWrite.setVx(v_x[iw]);
                //eventToWrite.setVy(v_y[iw]);
                
                Bottle tmp;
                tmp = eventToWrite.encode();
                clusterEvents.append(tmp);                      
            }
        }
        eventBottle dataTmp(&clusterEvents);
        evtCluster = dataTmp;
        
        //send it all out
        outPort.write();
    }
    //fprintf(stdout, "------------------------------------------------------------------\n");
}

//empty line to make gcc happy
