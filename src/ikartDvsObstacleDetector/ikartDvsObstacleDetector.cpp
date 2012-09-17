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
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <math.h>
#include <cv.h>

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

void image2world (int x, int y, double &d, double& ang)
{
	double ymin=1; //m
	double ymax=3.5; //m
	double xpixels = 128;
	double ypixels = 128;
	d=(y/ypixels*(ymax-ymin)) + ymin;
	ang=0;
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

void obstacleDetectorThread::updateIkartVel()
{
	Bottle *b_ikart_vel = port_ikart_velocity_input.read(false);
	if (b_ikart_vel)
	{
		ikart_vx = b_ikart_vel->get(0).asDouble();
		ikart_vy = b_ikart_vel->get(1).asDouble();
		ikart_vt = b_ikart_vel->get(2).asDouble();
		last_data = yarp::os::Time::now();
	}
	double curr_time = yarp::os::Time::now();
	if (curr_time - last_data > 0.3) 
	{
	//	printf ("Input command timeout! \n");
		ikart_vx = ikart_vy = ikart_vt = 0;
	}
}

void obstacleDetectorThread::compute_comparison()
{
	for (int y=0; y<128; y++)
		for (int x=0; x<128; x++)
		{
			double c = this->flow_model.output_ground_model_x[x][y] * measured_optical_flow_x[x][y] + 
			   	       this->flow_model.output_ground_model_y[x][y] * measured_optical_flow_y[x][y];

			compared_optical_flow_ang[x][y] = c;
		}
}

void obstacleDetectorThread::draw_comparison()
{
	IplImage *ipl=(IplImage*) comparison_image.getIplImage();
	int r =0;
	int g =0;
	int b =0;
	for (int y=0; y<ipl->height; y++)
    {
		uchar* p =(uchar*) ipl->imageData + y*ipl->widthStep;
		for(int x=0; x<ipl->width; x++)
		{
			double mag = sqrt (measured_optical_flow_x[x][y]*measured_optical_flow_x[x][y]+measured_optical_flow_y[x][y]*measured_optical_flow_y[x][y]);
			if (mag > 0.001)
			{
				if       (compared_optical_flow_ang[x][y]  > 0.01) {r = 0;   b= 0; g = 200;}
				else if  (compared_optical_flow_ang[x][y]  < 0.01) {r = 200; b= 0; g =0;   }
				else                                               {r = 200; b= 0; g =200; }
			}
			else  
			{
				r = 0; b= 0; g =0;
			}

			p[3*x+0]=r;
			p[3*x+1]=g;
		    p[3*x+2]=b;
		}
	}
}

void obstacleDetectorThread::used_simulated_measured_optical_flow()
{
	for (int y=0; y<128; y++)
		for (int x=0; x<128; x++)
		{
			measured_optical_flow_x[x][y]= this->flow_model.initialized_output_ground_model_x[x][y];
			measured_optical_flow_y[x][y]= this->flow_model.initialized_output_ground_model_y[x][y];
		}
}
void obstacleDetectorThread::draw_measured_check()
{
	static const yarp::sig::PixelMono16 black=0;
    static const yarp::sig::PixelMono16 white=255;
	static double scale = 100;
	static int c=BIGGER/2;
	
	IMGFOR(flow_measured_check_image ,i , j)
		{
			flow_measured_check_image(i, j) = 150;
		}

	for (int y=0, Y=0; y<N_PIXELS*BIGGER; y+=BIGGER, Y+=1)
		for (int x=0, X=0; x<N_PIXELS*BIGGER; x+=BIGGER, X+=1)
		{
			yarp::sig::draw::addSegment(flow_measured_check_image,black,x+c,y+c,int(x+measured_optical_flow_x[X][Y]*scale)+c,int(y+measured_optical_flow_y[X][Y]*scale)+c);
			if (fabs(measured_optical_flow_x[X][Y])> 0.0001 || fabs(measured_optical_flow_y[X][Y])> 0.0001)
				yarp::sig::draw::addCircle(flow_measured_check_image,black,x+int(measured_optical_flow_y[X][Y]*scale)+c,int(y+measured_optical_flow_y[X][Y]*scale)+c,2);
			else
				yarp::sig::draw::addCircle(flow_measured_check_image,black,x+int(measured_optical_flow_y[X][Y]*scale)+c,int(y+measured_optical_flow_y[X][Y]*scale)+c,1);
		}
}

void obstacleDetectorThread::run()
{
	//get the ikart velocity
	updateIkartVel();
	//updtae the model of the optical flow
	double speed_mult = 1000/this->getRate();
	speed_mult/=25; //<<<<<<<<<<<<<<<<<<<<<<<<
	flow_model.set_movement(-ikart_vx*speed_mult, -ikart_vy*speed_mult, -ikart_vt*speed_mult);
	flow_model.compute_model();

	//get the optical flow buffer;
	//VelocityBuffer* buff = port_buffered_optical_flow_input.read(false);
	Bottle *buff = port_optical_flow_input.read(false);
	if (buff)
	{
		int c = 0;
		int xdim = buff->get(c++).asInt();
		int ydim = buff->get(c++).asInt();
		//optical_flow_buffer = *buff;
		for (int y=0; y<128; y++)
			for (int x=0; x<128; x++)
			{
				c++;
				measured_optical_flow_x[x][y]= buff->get(c).asDouble();
				measured_optical_flow_y[x][y]= buff->get(c+16384).asDouble();
			}
	}
	else
	{
		for (int y=0; y<128; y++)
			for (int x=0; x<128; x++)
			{
				measured_optical_flow_x[x][y]= 0;
				measured_optical_flow_y[x][y]= 0;
			}
	}

	//used_simulated_measured_optical_flow();


	compute_comparison();

	//compute the output 'laser' scan 
	clearScan();
	double detected_distance = 1; //m
	compute_scan_1(detected_distance);

	//send the simulated obstacle
	if (port_simulated_scan_output.getOutputCount()>0)
	{
		yarp::sig::Vector &v=port_simulated_scan_output.prepare();
		v=scan_data;
		port_simulated_scan_output.write();
	}

	//send the optical flow model image
	if (port_flow_model_output.getOutputCount()>0)
	{
		flow_model.redraw();
		yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=port_flow_model_output.prepare();
		img=flow_model.flow_model_image;
		port_flow_model_output.write();
	}

	//send the comparison image
	if (port_comparison_output.getOutputCount()>0)
	{
		draw_comparison();
		yarp::sig::ImageOf<yarp::sig::PixelRgb>& img=port_comparison_output.prepare();
		img=comparison_image;
		port_comparison_output.write();
	}

	//send the calibration image
	if (port_calibration_output.getOutputCount()>0)
	{
		yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=port_calibration_output.prepare();
		img=flow_model.calibration_image;
		port_calibration_output.write();
	}

	//send the check image
	if (port_flow_measured_check_output.getOutputCount()>0)
	{
		draw_measured_check();
		yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=port_flow_measured_check_output.prepare();
		img=flow_measured_check_image;
		port_flow_measured_check_output.write();
	}
}

void obstacleDetectorThread::printStats()
{
}
