/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
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

#ifndef COMPASS_THREAD_H
#define COMPASS_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>

#include "VelocityBuffer.h"
#include "groundFlowModel.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace iCub::ctrl;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class obstacleDetectorThread: public yarp::os::RateThread
{
	public:

    protected:
    //configuration parameters
    double obstacle_size;   //m
	VelocityBuffer                   optical_flow_buffer;
	groundFlowModel                  flow_model;
	
	//ikart velocity data
	double                           ikart_vx;
	double                           ikart_vy;
	double                           ikart_vt;
	double                           last_data;

    //ports
	BufferedPort<Bottle>									  port_ikart_velocity_input;
	BufferedPort<VelocityBuffer>							  port_buffered_optical_flow_input;
	BufferedPort<yarp::sig::Vector>							  port_optical_flow_input;
    BufferedPort<yarp::sig::Vector>							  port_simulated_scan_output;
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > port_flow_model_output;
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    port_comparison_output;

	yarp::sig::ImageOf<yarp::sig::PixelMono16>                comparison_image;
    Property            iKartCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   scan_data;

    public:
    obstacleDetectorThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
		comparison_image.resize(128,128);
		scan_data.resize(1080,100);
		obstacle_size=1;
    }

	void set_model_params(double focal_lenght, double angle_deg, double height)
	{
	    flow_model.initialize(focal_lenght,angle_deg,height);
		flow_model.project_plane();
	}

    virtual bool threadInit()
    {
        //read configuration parametes


        //open module ports
		string localName = "/ikartDvsObstacleDetector";
		//port_optical_flow_input.open        ( (localName+"/flow:i").c_str() );
		port_buffered_optical_flow_input.open ( (localName+"/flow:i").c_str() );
		port_simulated_scan_output.open       ( (localName+"/scan:o").c_str() );
		port_ikart_velocity_input.open        ( (localName+"/ikart_velocity:i").c_str() );
		port_flow_model_output.open           ( (localName+"/flow_model_img:o").c_str() );
		port_comparison_output.open           ( (localName+"/flow_comparison_img:o").c_str() );

        //automatic port connections
        bool b = false;
        //b = Network::connect("/ikart_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
        //if (!b) {fprintf (stderr,"Unable to connect the port!"); return false;}

        return true;
    }

    virtual void run();
	void clearScan();
	void compute_scan_1(double detected_distance);
	void updateIkartVel();

    virtual void threadRelease()
    {    
		port_flow_model_output.interrupt();
		port_flow_model_output.close();
		port_ikart_velocity_input.interrupt();
		port_ikart_velocity_input.close();
        port_optical_flow_input.interrupt();
        port_optical_flow_input.close();
		port_buffered_optical_flow_input.interrupt();
		port_buffered_optical_flow_input.close();
	    port_simulated_scan_output.interrupt();
        port_simulated_scan_output.close();
		port_comparison_output.interrupt();
		port_comparison_output.close();
    }

    void printStats();

};

#endif
