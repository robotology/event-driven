#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <time.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

//#include "unmask.h"
#include "eventCodec.h"
#include "eventBottle.h"
#include "blob_tracker.h"
#include "tracker_pool.h"

//#include "/home/icub/Himanshu_local/iCubFiles/aquilaLite/libraries/iCubMotor/include/iCubMotor.h"
//#include "iCubMotor.h"

//using namespace aquilacubmotor;

int main(int numArgs, char** args)
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


  yarp::os::Network yarp;
  std::string filename = "output.txt";
  
  
	
  if(numArgs > 3){
    filename = args[3];
  }
  if(numArgs > 2){
    alpha_shape = atof(args[2]);
  }
  if(numArgs > 1){
    alpha_pos = atof(args[1]);
  }

	
  // File for outputing the data
  FILE *output_file;
  output_file = fopen (filename.c_str(), "w");
  if (output_file == NULL) 
    perror ("Error opening file");
  
  // We create the trackers
  TrackerPool tracker_pool_left(sig_x, sig_y, sig_xy, alpha_pos, alpha_shape, k, 
				max_dist, fixed_shape, tau_act, up_thresh, down_thresh, delete_thresh, 
				alpha_rep, d_rep, max_nb_trackers, nb_ev_reg);
  tracker_pool_left.set_collision_det_param(dist_thresh, vel_thresh, acc_thresh);

  TrackerPool tracker_pool_right(sig_x, sig_y, sig_xy, alpha_pos, alpha_shape, k, 
				 max_dist, fixed_shape, tau_act, up_thresh, down_thresh, delete_thresh,
				 alpha_rep, d_rep, max_nb_trackers, nb_ev_reg);
  tracker_pool_right.set_collision_det_param(dist_thresh, vel_thresh, acc_thresh);

  // Threshold for updating the position
  int min_nb_ev = 1;

  // We create the network

  

  // We create the port in which we will receive the DVS events
  yarp::os::BufferedPort<eventBottle> dvs_port;
  eventBottle *evBottle;

  // We set its name
  std::string local_dvs_port_name = "/dvsReadLocal:i";
  (dvs_port).open(local_dvs_port_name.c_str());
  printf("Created port %s to get data from DVS!!\n\n", local_dvs_port_name.c_str());

  // And we try to connect it to the port generating the events
  std::string remote_dvs_port_name = "/aexGrabber/eventBottle:o";
  bool connectionOk = yarp.connect(remote_dvs_port_name.c_str(), local_dvs_port_name.c_str());

  printf("Waiting for aexGrabber port to go alive!!!\n");	
  while (!connectionOk){
    connectionOk = yarp.connect(remote_dvs_port_name.c_str(), local_dvs_port_name.c_str());
  }
  printf("Connected to iCub's DVS port\n");

	
  // We create the ports that will send the images to YARP
  // Left eye
  yarp::os::Port image_port_left_out;
  std::string image_port_name_left_out = "/BlobTracker/imagePortLeft:o";
  image_port_left_out.open(image_port_name_left_out.c_str());
  // Right eye
  yarp::os::Port image_port_right_out;
  std::string image_port_name_right_out = "/BlobTracker/imagePortRight:o";
  image_port_right_out.open(image_port_name_right_out.c_str());

  // We create the images and give them their size
  // Output
  yarp::sig::ImageOf<yarp::sig::PixelRgb> gray_image;
  gray_image.resize(128,128);	
  gray_image.zero();
  for(int ii=0; ii<128; ii++){
    for(int jj=0; jj<128; jj++){
      gray_image.pixel(ii, jj) = yarp::sig::PixelRgb(127, 127, 127);
    }  
  }  

  
  // Left
  yarp::sig::ImageOf<yarp::sig::PixelRgb> image_left_out;
  image_left_out.resize(128,128);	
  image_left_out.zero();
  // Right
  yarp::sig::ImageOf<yarp::sig::PixelRgb> image_right_out;
  image_right_out.resize(128,128);	
  image_right_out.zero();

  // Define cluster init position.
  // Num clusters	= 5 (const init);
  const int numClusters = 5;
  // Cluster events = 0 (variable init);
  yarp::sig::Vector numEventsPerCluster;
  numEventsPerCluster.resize(5);
  yarp::sig::Vector currentEventNumbers, previousEventNumbers;
  currentEventNumbers.resize(128,128);

  // Variable controlling the loop
  bool run = true;

  // Variables that will hold the coordinates of the collisions
  std::vector<double> x_coll_left, y_coll_left, x_coll_right, y_coll_right;

  int numIters = 0;
  int confidenceVal = 0;
  bool moveEyes = false;
  int t_start_moving;
  double cen_x_prev, cen_y_prev; 
  
  int last_t_display = 0;
  int dt = 10000;	 

  while(run){ // && numIters > 10000000
    evBottle = dvs_port.read();
    numIters++;
    yarp::os::Bottle *bottle = (*evBottle).get_packet();
    emorph::ecodec::eEventQueue q;
    int sizeOfQ;
    if(bottle){
    	emorph::ecodec::eEvent::decode(*bottle, q);
        sizeOfQ = q.size();
    }
    else{
	sizeOfQ = 0;
    }
    
    //fprintf(output_file, "%d \n", sizeOfQ); 
        //printf("Received %i events \n", sizeOfQ);

    // If the number of events is lower than a predefined threshold, then we assume we are getting just noise, and we do nothing
    if(sizeOfQ > min_nb_ev){
      int ev_x, ev_y, pol, channel, ev_t;

      bool address_ev_found = false;
      bool ts_found = false;
      // We go through all the events
      for (int ii=0; ii < sizeOfQ; ii++){
	// We need the adress event first
	if (q[ii]->getType() == "AE"){
	  // We decode the event
	  emorph::ecodec::AddressEvent *ev = static_cast<emorph::ecodec::AddressEvent*>(q[ii]); 
	  if (ev->isValid()){
	    ev_x = 127-ev->getX();
	    ev_y = ev->getY();
	    pol = ev->getPolarity();
	    channel = ev->getChannel();
            fprintf(output_file, "%d\n", ((ev_x/4)*32+(ev_y/4))*(pol+1));
	    address_ev_found = true;
	  }
	}
	// Once the addres event has been found, we look for the timestamp
	if (q[ii]->getType() == "TS" && address_ev_found){
	  // We decode the event
	  emorph::ecodec::TimeStamp *ts = static_cast<emorph::ecodec::TimeStamp*>(q[ii]); 
	  if (ts->isValid()){
	    ev_t = ts->getStamp();
	    ts_found = true;
	  }
	}

	// If we have both the adress and the timestamp, we update the trackers
	if(address_ev_found && ts_found){
	  if(ev_x>=0 && ev_x <= 127 && ev_y>=0 && ev_y <= 127){
            if (channel == 0) {
	      if(pol == 1 | pol == 0){  
                tracker_pool_left.update(ev_x, ev_y, ev_t);
              }
              image_left_out.pixel(ev_y, ev_x) = yarp::sig::PixelRgb(255*pol, 255*pol, 255*pol);
	    }
	    else {
	      if(pol == 1 | pol == 0){
                tracker_pool_right.update(ev_x, ev_y, ev_t);
              }
              image_right_out.pixel(ev_y, ev_x) = yarp::sig::PixelRgb(255*pol, 255*pol, 255*pol);
	    }
          }
	  address_ev_found = false;
	  ts_found = false;
	  if(ev_t - last_t_display >= dt || ev_t - last_t_display < 0){
	    // We update the images of the ellipses
	    tracker_pool_left.display(image_left_out);
 	    tracker_pool_right.display(image_right_out);
            // Write the images into the yarp port
            image_port_left_out.write(image_left_out);
            image_port_right_out.write(image_right_out);
            // And reset them to gray
            image_left_out = gray_image;
            image_right_out = gray_image;
	    last_t_display = ev_t;
	  }
	}
      }

      double mean_x = 0;
      double mean_y = 0;
      //bool collision = false;

      // We get the data from the tracker pool
      tracker_pool_left.get_collisions(x_coll_left, y_coll_left);
      tracker_pool_right.get_collisions(x_coll_right, y_coll_right);
      int nb_coll = x_coll_left.size() + x_coll_right.size();

      if(nb_coll>0){
        moveEyes = true;
        for(int ii=0; ii<x_coll_left.size(); ii++){
          mean_x =+ x_coll_left[ii];
          mean_y =+ y_coll_left[ii];
        }
        for(int ii=0; ii<x_coll_right.size(); ii++){
          mean_x =+ x_coll_right[ii];
          mean_y =+ y_coll_right[ii];
        }
	mean_x/=nb_coll;
	mean_y/=nb_coll;
      }
            
      
    }
  }
  fclose(output_file);
}
