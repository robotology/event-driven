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

    /* create the thread and pass pointers to the module parameters */
    eventBottleManager = new EventBottleManager( moduleName );

    /*
    * set the file name for saving cluster data
    */
    fileName = rf.check("filename", 
                        Value("clusters.txt"), 
                        "file name (string)").asString();
    fprintf(stdout,"output file %s \n", fileName.c_str());
    eventBottleManager ->setFileName(fileName);

    /*
    * set the alpha shape of the gauss cluster
    */
    alphaShape = rf.check("alpha_shape", 
                        Value(10), 
                        "alpha shape (int)").asInt();
    fprintf(stdout,"alpha shape %d \n", alphaShape);
    eventBottleManager ->setAlphaShape(alphaShape);

    /*
    * set the alpha pos of the gauss cluster
    */
    alphaPos = rf.check("alpha_pos", 
                        Value(10), 
                        "alpha pos (int)").asInt();
    fprintf(stdout,"alpha pos %d \n", alphaPos);
    eventBottleManager ->setAlphaPos(alphaPos);

    /* initialize variables */
    eventBottleManager->init();
 

   /* now open the manager to do the work */
    eventBottleManager->open();

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
EventBottleManager::EventBottleManager( const string &moduleName )
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
}
/**********************************************************/
bool EventBottleManager::init()
{

    // Initial parameters of the Blob Tracker and the Tracker Pool
    double sig_x = 5;
    double sig_y = 5;
    double sig_xy = 0;
    double alpha_pos = 0.1;
    double alpha_shape = 0.01;
    double max_dist = 30;
    double k = 2;
    bool fixed_shape = false;
    double tau_act = 10000;
    double up_thresh = 5;
    double down_thresh = 1;
    double delete_thresh = 0.00000001;
    double alpha_rep = 2;
    int d_rep = 40;
    int max_nb_trackers = 40;
    int nb_ev_reg = 50;
    double dist_thresh = 30;
    double vel_thresh = 50;
    double acc_thresh = 300;

    // File for outputing the data
    FILE *output_file;
    output_file = fopen (fileName.c_str(), "w");
    if (output_file == NULL) 
        perror ("Error opening file");
  
    // Create the trackers
    //TrackerPool tracker_pool_left(sig_x, sig_y, sig_xy, alpha_pos, alpha_shape, k, max_dist, fixed_shape, tau_act, up_thresh, down_thresh, delete_thresh, alpha_rep, d_rep, max_nb_trackers, nb_ev_reg);
    //tracker_pool_left.set_collision_det_param(dist_thresh, vel_thresh, acc_thresh);

    //TrackerPool tracker_pool_right(sig_x, sig_y, sig_xy, alpha_pos, alpha_shape, k, 
		//		 max_dist, fixed_shape, tau_act, up_thresh, down_thresh, delete_thresh,
		//		 alpha_rep, d_rep, max_nb_trackers, nb_ev_reg);
    //tracker_pool_right.set_collision_det_param(dist_thresh, vel_thresh, acc_thresh);

    // Threshold for updating the position
    int min_nb_ev = 1;

  // We create the images and give them their size
  // Output
    ImageOf<PixelRgb> gray_image;
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
    ImageOf<PixelRgb> image_left_out;
    image_left_out.resize(128,128);	
    image_left_out.zero();
  // Right
    ImageOf<PixelRgb> image_right_out;
    image_right_out.resize(128,128);	
    image_right_out.zero();

  // Define cluster init position.
  // Num clusters	= 5 (const init);
    const int numClusters = 5;
  // Cluster events = 0 (variable init);
    Vector numEventsPerCluster;
    numEventsPerCluster.resize(5);
    Vector currentEventNumbers, previousEventNumbers;
    currentEventNumbers.resize(128,128);

  // Variables that will hold the coordinates of the collisions
    vector<double> x_coll_left, y_coll_left, x_coll_right, y_coll_right;

    int numIters = 0;
    int confidenceVal = 0;
    bool moveEyes = false;
    int t_start_moving;
    double cen_x_prev, cen_y_prev; 
  
    int last_t_display = 0;
    int dt = 10000;	 

    return true;
}

/**********************************************************/
bool EventBottleManager::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + "/eventBottle:i";
    inPort.open( inPortName.c_str() );

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

    inPort.close();
    leftPort.close();
    rightPort.close();
    outPort.close();
    
    fprintf(stdout,"finished closing the read port...\n");
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
    unsigned long ts;
    //decode packet
    Bottle event; // for eventBottle port eventPort

    eventBottle &evt = outPort.prepare();
    
    if(eEvent::decode(*bot.get_packet(),q)) 
    {   
        int size = q.size();
        for (int e = 0; e < size; e++)
        {
            if(q[e] != 0)
            {
                if(q[e]->getType()=="TS") //identify the type of the packet (Time Stamp)
                {
                    TimeStamp* ptr=dynamic_cast<TimeStamp*>(q[e]);
                    ts = (unsigned long) ptr->getStamp();
                    //fprintf(stdout, "Size %d TimeStamp: %d", size, (unsigned int) ptr->getStamp());
                }
                else if (q[e]->getType()=="AE") //identify the type of the packet (Address Event)
                {
                    AddressEvent* ptr=dynamic_cast<AddressEvent*>(q[e]); //create the Address Event for the type
                    
                                        
                    if(ptr->isValid()) 
                    {   // get data from AE
                        short posX = ptr->getX();
                        short posY = ptr->getY();
                        short polarity = ptr->getPolarity();
                        short channel = ptr->getChannel();
                        
                        ClusterEventGauss cluEvt; //create the Cluster Event 
                        //fill the CLE-G with data:
                        cluEvt.setChannel(channel);
                        //  cluEvt->setId(1); 
                        cluEvt.setXCog(posX); 
                        cluEvt.setYCog(posY);
                        
                        int numAE = 0;
                        int xS2 = 1;
                        int yS2 = 2;
                        int xyS = 3;
                        
                        cluEvt.setNumAE(numAE); 
                        cluEvt.setXSigma2(xS2);
                        cluEvt.setYSigma2(yS2);
                        cluEvt.setXYSigma(xyS);
                        
                        //create the timestamp and fill it
                        TimeStamp time;
                        time.setStamp(ts);
                        
                        //create a bottle with ts and cle-g
                        Bottle tmpEv;
                        tmpEv = time.encode();
                        tmpEv.append(cluEvt.encode());
                        
                        //append bottle tmpEv to the bottle of events
                        event.append(tmpEv);
                                                
                        string type = cluEvt.getType();

                        //fprintf(stdout, "Size %d TimeStamp: %d Event: (Type: %s X: %d  Y: %d  pol: %d  cha: %d m: %d xs: %d ys: %d xys: %d) \n",size, ts, type.c_str(), posX, posY, polarity, channel, numAE, xS2, yS2, xyS);
                        
                    } 
                }
            }
        } 

        //create and fill in dataTmp (eventBottle) with data from event (Bottle) 
        //fprintf(stdout, "\n\n\nEvent: %s\n", event.toString().c_str());
        eventBottle dataTmp(&event);
        //copy dataTmp to eventBottle out
        evt = dataTmp;
        //send it all out

        outPort.write();
    }   
    //fprintf(stdout, "------------------------------------------------------------------\n");
}

//empty line to make gcc happy
