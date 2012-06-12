// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email:shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file dvsCalibratorThread.cpp
 * @brief Implementation of the thread (see header dvsCalibratorThread.h)
 */

#ifndef _FITTING_MODEL_CPP
#define _FIITING_MODEL_CPP




#include <iCub/fittingModel.h>


void setThetasOfPolarCoord(double* valueArray)
{
    for(int i=0; i<N ; ++i)
    {
        thetasOfPolarCoord[i] = Xcoord[i];//valueArray[i];
    }
    modelReady = true;
}
int modelFunction(const gsl_vector * x, void *dataE, gsl_vector * f)
{
    size_t n = ((struct dataE *)dataE)->n;
    double *y = ((struct dataE *)dataE)->y;
    double *sigma = ((struct dataE *) dataE)->sigma;

    /* Parameters are (r0,phi, K1, K2) */

    double r0 = gsl_vector_get (x, 0);
    double phi = gsl_vector_get (x, 1);
    double K1 = gsl_vector_get (x, 2)*SCALE_K1;
    //double K2 = gsl_vector_get (x, 3)*SCALE_K2;

    size_t i;

    for (i = 0; i < n; i++)
    {
        /* Model Yi = A * exp(-lambda * i) + b */
        /* Model Yi = z(1+K1.z^2 + K2.z^4) */

        double t = i;
        double z = r0/cos(Xcoord[i] - phi);
        double Yi =z*(1.0+K1*z);//*z + K2*z*z*z*z);
        gsl_vector_set (f, i, (Yi - y[i])*ERROR_FACTOR);///sigma[i]);
    }

    return GSL_SUCCESS;
}

int derivativeModelFunction(const gsl_vector * x, void *dataE,
                            gsl_matrix * J)
{
    size_t n = ((struct dataE *)dataE)->n;
    double *sigma = ((struct dataE *) dataE)->sigma;

    /* Parameters are (r0,phi, K1, K2) */
    double r0 = gsl_vector_get (x, 0);
    double phi = gsl_vector_get (x, 1);
    double K1 = gsl_vector_get (x, 2)*SCALE_K1;
    //double K2 = gsl_vector_get (x, 3)*SCALE_K2;



    for (size_t i = 0; i < n; i++)
    {
        /* Jacobian matrix J(i,j) = dfi / dxj, */
        /* where fi = (Yi - yi)*ERROR_FACTOR,      */
        /* and the xj are the parameters (r0,phi, K1, K2)*/
        double t = i;
        double s = sigma[i];
        double cosineOfDiff = cos(Xcoord[i] - phi);
        double z = r0/cosineOfDiff;
        double delFbydelZ = (1.0 + 2.0*K1*z);//*z + 5.0*K2*z*z*z*z);
        gsl_matrix_set (J, i, 0, ERROR_FACTOR*delFbydelZ/cosineOfDiff);
        gsl_matrix_set (J, i, 1, ERROR_FACTOR*delFbydelZ*r0*sin(Xcoord[i] - phi)*(-1.0/(cosineOfDiff*cosineOfDiff)));
        gsl_matrix_set (J, i, 2, ERROR_FACTOR*z*z);//*z);
        //gsl_matrix_set (J, i, 3, ERROR_FACTOR*z*z*z*z*z);
    }
    return GSL_SUCCESS;
}

int modelFunctionAndDerivative(const gsl_vector * x, void *dataE,
                               gsl_vector * f, gsl_matrix * J)
{
    modelFunction (x, dataE, f);
    derivativeModelFunction (x, dataE, J);

    return GSL_SUCCESS;
}







#endif



