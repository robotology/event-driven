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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <math.h>

#include "ikartDVSObstacleDetector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

void obstacleDetectorThread::clearScan()
{
	for (int i=0; i<1080; i++)
	{
		scan_data[i]=100;
	}
}

int deg2las (double deg)
{
	return (int)(deg/270*1080);
}

double las2deg (int las)
{
	return double(las)/1080*270;
}

void obstacleDetectorThread::compute_scan_1(double detected_distance)
{
	double alpha     = atan2(obstacle_size/2,detected_distance)*180.0/M_PI;
	int alpha_i      = deg2las(alpha);
	int alpha_start  = 1080/2-alpha_i;
	int alpha_end    = 1080/2+alpha_i;
	for (int i=0; i<alpha_i*2; i++)
	{
		int index = alpha_start+i;
		double t = 270/2-las2deg(index);
		double coeff = cos(t/180.0*M_PI);
		double curr_d = detected_distance / coeff;
		scan_data[index] = curr_d;
		//fprintf (stdout, "%d %f\n", i, scan_data[index]);
	}
}

void obstacleDetectorThread::run()
{
	//get the optical flow buffer;
	VelocityBuffer* buff = port_buffered_optical_flow_input.read(false);
	if (buff)
	{
		optical_flow_buffer = *buff;
	}

	//compute the scan 
	clearScan();
	double detected_distance = 1; //m
	compute_scan_1(detected_distance);

	//send the simulate obstacle
	if (port_simulated_scan_output.getOutputCount()>0)
	{
		yarp::sig::Vector &v=port_simulated_scan_output.prepare();
		v=scan_data;
		port_simulated_scan_output.write();
	}
}

void obstacleDetectorThread::printStats()
{
}
