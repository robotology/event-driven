/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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


#ifndef LKLOCALFLOW_H_
#define LKLOCALFLOW_H_

#include <math.h>
#include "MyMatrix.h"
#include "param.h"

class LKLocalFlow{

	POLARITY_TYPE SOBEL_X [3][3];
	POLARITY_TYPE SOBEL_Y [3][3];

    int neighborRadius;
    int windowLength;

    double lambda;

    double * GuaWeight;

    void eigenVector_symm2x2(double a11, double a12, double a22, double eigValue, double * eigVector);
    void pinv_symm2x2(double a11, double a12 ,double a22, MyMatrix<double> & invA);

    void LeastSquareSolver(MyMatrix<double> A, MyMatrix<double> b, double * x);

public:

    LKLocalFlow(int nRadius);
    ~LKLocalFlow();

    void setGuaWeights(double stdDev);

    void calVelocity(/*TIMESTAMP_TYPE*/double  tsDiff, MyMatrix<TIMESTAMP_TYPE> * ts, MyMatrix<POLARITY_TYPE> * It, MyMatrix<POLARITY_TYPE> * Ipt,  int X, int  Y, double *);
};


#endif /* LOCALFLOW_H_ */
