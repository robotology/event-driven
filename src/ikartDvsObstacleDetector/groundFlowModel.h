/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#ifndef GROUND_FLOW_MODEL_H
#define GROUND_FLOW_MODEL_H

#include <math.h>
#include <iCub/ctrl/math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <string>

using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265
#endif

//#define N_PIXELS 128 
#define N_PIXELS 6

#ifndef IMGFOR
#define IMGFOR(img,i,j) for (int i=0; i<(img).width(); i++) for (int j=0; j<(img).height(); j++)
#endif

class groundFlowModel
{
	private:
	double input_ground_model_y [N_PIXELS][N_PIXELS];
	double input_ground_model_x [N_PIXELS][N_PIXELS];
	double output_ground_model_y [N_PIXELS][N_PIXELS];
	double output_ground_model_x [N_PIXELS][N_PIXELS];

	Matrix tx_matrix;
	Matrix p_matrix;

	public:
	yarp::sig::ImageOf<yarp::sig::PixelMono16> flow_model_image;

	void redraw()
	{	
		static const yarp::sig::PixelMono16 black=0;
        static const yarp::sig::PixelMono16 white=255;

		IMGFOR(flow_model_image ,i , j)
		{
			flow_model_image(i, j) = 150;
		}

		//yarp::sig::draw::addSegment(flow_model_image,black,X,Y,hx,hy);
        //yarp::sig::draw::addCircle(flow_model_image,black,hx,hy,2);
	}

	groundFlowModel()
	{
		flow_model_image.resize(4*N_PIXELS,4*N_PIXELS);

		for (int x=0; x<N_PIXELS; x++)
			for (int y=0; y<N_PIXELS; y++)
				input_ground_model_x[x][y]=0;

		for (int x=0; x<N_PIXELS; x++)
			for (int y=0; y<N_PIXELS; y++)
				input_ground_model_y[x][y]=1;

		double ang = -135.0/180.0*M_PI;
		tx_matrix.resize(4,4);
		tx_matrix.zero();
		/*
		tx = 1   0   0   x
		     0   c  -s   y
			 0   s   c   z
			 0   0   0   1
		*/
		//x rotation
		tx_matrix[0][0] = 1;
		tx_matrix[1][1] = cos(ang);
		tx_matrix[2][2] = cos(ang);
		tx_matrix[1][2] = -sin(ang);
		tx_matrix[2][1] = sin(ang);
		tx_matrix[0][3] = 0; //x
		tx_matrix[1][3] = 0; //y
		tx_matrix[2][3] = 1; //z
		tx_matrix[3][3] = 1; 
		tx_matrix = iCub::ctrl::SE3inv(tx_matrix);

	//	printf ("%s /n",tx_matrix.toString().c_str());
		/*
		//y rotation
		tx_matrix[0][0] = cos(ang);
		tx_matrix[1][1] = cos(ang);
		tx_matrix[1][0] = sin(ang);
		tx_matrix[0][1] = -sin(ang);
		tx_matrix[2][2] = 1;
		tx_matrix[0][3] = 0; //x
		tx_matrix[1][3] = 0; //y
		tx_matrix[2][3] = 1; //z
		tx_matrix[3][3] = 1; */

		/*
		// z rotation
		tx_matrix[0][0] = cos(ang);
		tx_matrix[1][1] = cos(ang);
		tx_matrix[1][0] = sin(ang);
		tx_matrix[0][1] = -sin(ang);
		tx_matrix[2][2] = 1;
		tx_matrix[0][3] = 0; //x
		tx_matrix[1][3] = 0; //y
		tx_matrix[2][3] = 1; //z
		tx_matrix[3][3] = 1; */


		p_matrix.resize(3,4);
		p_matrix.zero();
		double f = 1;
		double c = 10;//N_PIXELS/2;
		p_matrix[0][0] = f;
		p_matrix[0][2] = 160;
		p_matrix[1][1] = f;
		p_matrix[1][2] = 120;
		p_matrix[2][2] = 1;
		/*
		p = f 0 c 0
		    0 f c 0
			0 0 1 0
		*/


		for (int x=0; x<N_PIXELS; x++)
			{
				for (int y=0; y<N_PIXELS; y++)
				{
					Vector point_in1;
					Vector point_out1;
					Vector point_in2;
					Vector point_out2;
					point_in1.resize(4);
					point_in1[0]=x; 
					point_in1[1]=y; 
					point_in1[2]=0; 
					point_in1[3]=1; 
					point_in2.resize(4);
					point_in2[0]=x+input_ground_model_x[x][y]; 
					point_in2[1]=y+input_ground_model_y[x][y]; 
					point_in2[2]=0; 
					point_in2[3]=1; 

				    printf("in1 %s\n", point_in1.toString().c_str());
					//printf("%s\n", point_in2.toString().c_str());

					point_out1 = tx_matrix * point_in1;
					printf("T1 %s\n", point_out1.toString().c_str());
					point_out1 = p_matrix  * point_out1;
					point_out1 = (1.0/point_out1[2]) * point_out1 ;
					printf("P1 %s\n", point_out1.toString().c_str());
					
					printf("in2 %s\n", point_in2.toString().c_str());
					point_out2 = tx_matrix * point_in2;
					printf("T2 %s\n", point_out2.toString().c_str());
					point_out2 = p_matrix  * point_out2;
					point_out2 = (1.0/point_out2[2]) * point_out2 ;
					printf("P2 %s\n\n", point_out2.toString().c_str());

					output_ground_model_x[x][y] = point_out2[0]-point_out1[0];
					output_ground_model_y[x][y] = point_out2[1]-point_out1[1];			
				}
			}

		for (int x=0; x<N_PIXELS; x++)
		{
			for (int y=0; y<N_PIXELS; y++)
			{printf ("%+4.4f     ", output_ground_model_x[x][y]);}
			 printf ("\n");
		}

		printf ("----------\n");
		for (int x=0; x<N_PIXELS; x++)
		{
			for (int y=0; y<N_PIXELS; y++)
			{printf ("%+4.4f     ", output_ground_model_y[x][y]);}
			 printf ("\n");
		}
	}



};

#endif