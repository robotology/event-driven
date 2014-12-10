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

#ifndef FITTING_MODEL_H
#define FIITING_MODEL_H



#include <stdlib.h>
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
 
 
#define N 86
#define NBR_PARAMETERS 3
#define ERROR_FACTOR .1
#define TOLERANCE_DERIVATIVE_FN 1e-2
#define TOLERANCE_FN 1e-3
#define SCALE_K1 1.0 // /(64.0*64.0)
#define SCALE_K2 SCALE_K1*SCALE_K1

struct dataE {
size_t n;
double * y;
double * sigma;
};

 
static double thetasOfPolarCoord[N];
static double Xcoord[N];
static double Ycoord[N];
static bool modelReady;

void setThetasOfPolarCoord(double* valueArray);
int modelFunction(const gsl_vector * x, void *dataE, gsl_vector * f);
 
int derivativeModelFunction(const gsl_vector * x, void *dataE, 
      gsl_matrix * J);
 
int modelFunctionAndDerivative(const gsl_vector * x, void *dataE,
       gsl_vector * f, gsl_matrix * J);
       



     
#endif



