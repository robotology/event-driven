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
#include <yarp/math/SVD.h>
#include <string>

using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define N_PIXELS 128 
//#define N_PIXELS 50

#ifndef IMGFOR
#define IMGFOR(img,i,j) for (int i=0; i<(img).width(); i++) for (int j=0; j<(img).height(); j++)
#endif

#define  BIGGER 8
#define  STEP   4

class groundFlowModel
{
	private:
	double input_grid_matrix_y [N_PIXELS][N_PIXELS];
	double input_grid_matrix_x [N_PIXELS][N_PIXELS];
	double input_grid_matrix_d [N_PIXELS][N_PIXELS];
	double input_grid_matrix_t [N_PIXELS][N_PIXELS];

	double input_ground_model_y [N_PIXELS][N_PIXELS];
	double input_ground_model_x [N_PIXELS][N_PIXELS];

	Matrix tx_matrix;
	Matrix p_matrix;

	public:
	double model_ang_deg;
	double model_f;
	double model_height;
	double k_off;
	yarp::sig::ImageOf<yarp::sig::PixelMono16> flow_model_image;
	yarp::sig::ImageOf<yarp::sig::PixelMono16> calibration_image;
	double output_ground_model_y [N_PIXELS][N_PIXELS];
	double output_ground_model_x [N_PIXELS][N_PIXELS];

	double initialized_output_ground_model_y [N_PIXELS][N_PIXELS];
	double initialized_output_ground_model_x [N_PIXELS][N_PIXELS];

	void redraw()
	{	
		static const yarp::sig::PixelMono16 black=0;
        static const yarp::sig::PixelMono16 white=255;

		IMGFOR(flow_model_image ,i , j)
		{
			flow_model_image(i, j) = 150;
		}

		int c=BIGGER/2;
		//double scale = 300;
		double scale = 1;
		for (int y=0, Y=0; y<N_PIXELS*BIGGER; y+=BIGGER*4, Y+=4)
			for (int x=0, X=0; x<N_PIXELS*BIGGER; x+=BIGGER*4, X+=4)
			{
				//yarp::sig::draw::addSegment(flow_model_image,black,x,y,x+20,y+20);
#if DEBUG
				printf ("x:%d y:%d X:%d Y:%d %d %d %f %f\n", x, y, X, Y, int(x+output_ground_model_x[X][Y]*scale), int(y+output_ground_model_y[X][Y]*scale), output_ground_model_x[X][Y], output_ground_model_y[X][Y]);
#endif
				yarp::sig::draw::addSegment(flow_model_image,black,x+c,y+c,int(x+output_ground_model_x[X][Y]*scale)+c,int(y+output_ground_model_y[X][Y]*scale)+c);
				yarp::sig::draw::addCircle(flow_model_image,black,x+int(output_ground_model_x[X][Y]*scale)+c,int(y+output_ground_model_y[X][Y]*scale)+c,2);
			}
#if DEBUG
		printf ("-----------\n");
#endif
	}

	void project_plane()
	{
		Vector plane_vector(4);
		plane_vector[0] = 0;
		plane_vector[1] = 0;
		plane_vector[2] = 1;
		plane_vector[3] = 1;

		Vector n(3);
		n[0]=plane_vector[0];
		n[1]=plane_vector[1];
		n[2]=plane_vector[2];

		Vector orig(4,0.0);
		orig[0] = 0;
		orig[1] = 0;
		orig[2] = 0;
		orig[3] = 1;

		double z = 1.0;
		int u = 0;
		int v = 0;
		Matrix invPrj = Matrix(yarp::math::pinv(p_matrix.transposed()).transposed());
		for (u=0; u< N_PIXELS; u++)
			for (v=0; v< N_PIXELS; v++)
			{
				Vector p(3);
				p[0]=z*u;
				p[1]=z*v;
				p[2]=z;

				Vector xe=invPrj*p;
				xe[3]=1.0;  // impose homogeneous coordinates
				//printf ("xe %d %d >>> %s\n",u,v,xe.toString().c_str());

				Vector ray=(iCub::ctrl::SE3inv(tx_matrix)*xe).subVector(0,2);
				//printf ("ray %d %d >>> %s\n",u,v,ray.toString().c_str());

				Vector p0(3,0.0);
				/*
				if (plane_vector[0]!=0.0)
					p0[0]=-plane_vector[3]/plane_vector[0];
				else if (plane_vector[1]!=0.0)
					p0[1]=-plane_vector[3]/plane_vector[1];
				else if (plane_vector[2]!=0.0)
					p0[2]=-plane_vector[3]/plane_vector[2];
				*/
				//printf ("p0 %d %d >>> %s\n",u,v,p0.toString().c_str());

				Vector e=(iCub::ctrl::SE3inv(tx_matrix)*orig).subVector(0,2); 

				//printf ("e %d %d >>> %s\n",u,v,e.toString().c_str());

				// compute the projection
				Vector vray=ray-e;
				//printf ("vray=ray-e %d %d >>> %s\n",u,v,vray.toString().c_str());
				//printf ("n %d %d >>> %s\n",u,v,n.toString().c_str());
				Vector test2 = p0-e;
				//printf ("p0-e %d %d >>> %s\n",u,v,test2.toString().c_str());

				Vector result=e+(dot(p0-e,n)/dot(vray,n))*vray;

				//double test = dot(vray,n);
				//printf ("dot %d %d >>> %f\n",u,v,test);
				//printf ("result %d %d >>> %s\n",u,v,result.toString().c_str());
				//printf ("r-----------------\n");
				input_grid_matrix_x[u][v] = result[0];
				input_grid_matrix_y[u][v] = result[1];
				input_grid_matrix_d[u][v] = sqrt(result[0]*result[0]+ result[1]* result[1]);
				input_grid_matrix_t[u][v] = atan2(result[1],result[0]);
			}
			        
	}

	void initialize (double focal_lenght, double angle_deg, double height)
	{
#define ZOOM 8
		flow_model_image.resize(BIGGER*N_PIXELS,BIGGER*N_PIXELS);
		calibration_image.resize(ZOOM*N_PIXELS,ZOOM*N_PIXELS);
		
		model_ang_deg = angle_deg;
		double ang = model_ang_deg/180.0*M_PI;
		model_height = height;
		model_f = focal_lenght;
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
		tx_matrix[2][3] = height; //z
		tx_matrix[3][3] = 1; 
		tx_matrix = iCub::ctrl::SE3inv(tx_matrix);

		p_matrix.resize(3,4);
		p_matrix.zero();
		double f = focal_lenght;
		double c = N_PIXELS/2;
		p_matrix[0][0] = f;
		p_matrix[0][2] = c;
		p_matrix[1][1] = f;
		p_matrix[1][2] = c;
		p_matrix[2][2] = 1;
		/*
		p = f 0 c 0
		    0 f c 0
			0 0 1 0
		*/

		calibration_image.zero();
		static const yarp::sig::PixelMono16 black=0;
        static const yarp::sig::PixelMono16 white=255;

		Vector istart1; istart1.resize(4);
		Vector iend1; iend1.resize(4);
		Vector ostart1; ostart1.resize(3);
		Vector oend1; oend1.resize(3);
		Vector istart2; istart2.resize(4);
		Vector iend2; iend2.resize(4);
		Vector ostart2; ostart2.resize(3);
		Vector oend2; oend2.resize(3);
		for (int i=0; i<1000; i++)
		{
			double sq_size = 0.045; //9 centimeters for the large chess board, 0.045 just for display
			istart1[0]=-10; istart1[1]=sq_size*i+k_off; istart1[2]=0; istart1[3]=1;
			iend1[0]=10;   iend1[1]=sq_size*i+k_off;   iend1[2]=0;   iend1[3]=1;
			istart2[0]=sq_size*(i-500); istart2[1]=-100; istart2[2]=0; istart2[3]=1;
			iend2[0]=sq_size*(i-500);   iend2[1]=0;   iend2[2]=0;   iend2[3]=1;

			ostart1 = tx_matrix*istart1;
			ostart1 = p_matrix*ostart1;
			ostart1 = (1.0/ostart1[2]) * ostart1 ;
			oend1 = tx_matrix*iend1;
			oend1 = p_matrix*oend1;
			oend1 = (1.0/oend1[2]) * oend1 ;

			ostart2 = tx_matrix*istart2;
			ostart2 = p_matrix*ostart2;
			ostart2 = (1.0/ostart2[2]) * ostart2 ;
			oend2 = tx_matrix*iend2;
			oend2 = p_matrix*oend2;
			oend2 = (1.0/oend2[2]) * oend2 ;

			//printf ("start   %f %f ---> %f  %f\n", istart1[0], istart1[1], ostart1[0], ostart1[1]);
			//printf ("end     %f %f ---> %f  %f\n", iend1[0], iend1[1], oend1[0], oend1[1]);
			yarp::sig::draw::addSegment(calibration_image,white,(int)(ostart1[0]*ZOOM),(int)(ostart1[1]*ZOOM),(int)(oend1[0]*ZOOM),(int)(oend1[1]*ZOOM));
			yarp::sig::draw::addSegment(calibration_image,white,(int)(ostart2[0]*ZOOM),(int)(ostart2[1]*ZOOM),(int)(oend2[0]*ZOOM),(int)(oend2[1]*ZOOM));
		}
	}

	void set_movement(double x_vel, double y_vel, double t_vel)
	{
		for (int x=0; x<N_PIXELS; x++)
			for (int y=0; y<N_PIXELS; y++)
				{
					t_vel=0; //<<<<<<<<<<<<<<<<<<<
					input_ground_model_x[x][y]=x_vel+t_vel/180*M_PI*input_grid_matrix_d[x][y]*sin(input_grid_matrix_t[x][y]);
					input_ground_model_y[x][y]=y_vel-t_vel/180*M_PI*input_grid_matrix_d[x][y]*cos(input_grid_matrix_t[x][y]);
				}
	}

	void compute_model()
	{
		for (int x=0; x<N_PIXELS; x++)
		{
			for (int y=0; y<N_PIXELS; y++)
			{
				Vector point_in1;
				Vector point_out1;
				Vector point_in2;
				Vector point_out2;
				point_in1.resize(4);
				point_in1[0]=input_grid_matrix_x[x][y]; 
				point_in1[1]=input_grid_matrix_y[x][y]; 
				point_in1[2]=0; 
				point_in1[3]=1; 
				point_in2.resize(4);
				point_in2[0]=point_in1[0]+input_ground_model_x[x][y]; 
				point_in2[1]=point_in1[1]+input_ground_model_y[x][y]; 
				point_in2[2]=0; 
				point_in2[3]=1; 

				//printf("in1 %s\n", point_in1.toString().c_str());
				//printf("%s\n", point_in2.toString().c_str());

				point_out1 = tx_matrix * point_in1;
				//printf("T1 %s\n", point_out1.toString().c_str());
				point_out1 = p_matrix  * point_out1;
				point_out1 = (1.0/point_out1[2]) * point_out1 ;
				//printf("P1 %s\n", point_out1.toString().c_str());
					
				//printf("in2 %s\n", point_in2.toString().c_str());
				point_out2 = tx_matrix * point_in2;
				//printf("T2 %s\n", point_out2.toString().c_str());
				point_out2 = p_matrix  * point_out2;
				point_out2 = (1.0/point_out2[2]) * point_out2 ;
				//printf("P2 %s\n\n", point_out2.toString().c_str());

				output_ground_model_x[x][y] = point_out2[0]-point_out1[0];
				output_ground_model_y[x][y] = point_out2[1]-point_out1[1];	
#if TEST_PROJ
				output_ground_model_x[x][y] = point_out2[0];
				output_ground_model_y[x][y] = point_out2[1];		
#endif 
			}
		}
	}
	groundFlowModel()
	{
		k_off=0;
		//initialize(62.5,-135,0.6);
		initialize(680,-122,0.6);
		project_plane();
		//1m/s = 0.001m/ms = 0,030m/30ms
		set_movement (0, 0.001, 0.0);
		compute_model();

		for (int y=0; y<128; y++)
		for (int x=0; x<128; x++)
		{
			initialized_output_ground_model_x[x][y] = output_ground_model_x[x][y];
			initialized_output_ground_model_y[x][y] = output_ground_model_y[x][y];
		}

		printf ("----------\n");
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