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
 * @file LMCurveFit.h
 * @brief This class is for implementing Levenberg–Marquardt algorithm for N dimensional non-linear curve fitting, utilising GNU Scientific Library
 */
 
 #ifndef _LM_CURVE_FIT_H
 #define _LM_CURVE_FIT_H
 
 #include <stdlib.h>
 #include <stdio.h>
 #include <gsl/gsl_rng.h>
 #include <gsl/gsl_randist.h>
 #include <gsl/gsl_vector.h>
 #include <gsl/gsl_blas.h>
 #include <gsl/gsl_multifit_nlin.h>
 
 #include <boost/bind.hpp>
 #include <boost/function.hpp>
 
 
 
 #define LM_NBR_OF_POINTS 40
 #define LM_MAX_ITERATIONS 500


struct data {
       size_t n;
       double * y;
       double * sigma;
     };
    
class LMCurveFit{
		const gsl_multifit_fdfsolver_type *T;
       	gsl_multifit_fdfsolver *s;
       	int status;
       	unsigned int iterations;
       	size_t nbrOfDataPoints;
       	size_t dimensionOfData;
       	gsl_matrix *covar;
       	data dataStruct;
       	gsl_multifit_function_fdf f;
       	double* Y; 										// value of function at various points
       	double* sigma;									// noise
       	gsl_vector_view x;
       	const gsl_rng_type * type;
        gsl_rng * r;
        
  public:
        LMCurveFit();
        LMCurveFit(int dim, int pts, double* yVal, double* sigVal);
        
        ~LMCurveFit();
        void initializeLM();
        int solveLM();
        void print_state (size_t iter, gsl_multifit_fdfsolver * s);
        
        
        gsl_multifit_fdfsolver* getMultiFitSolver()  {       return s;    };

};

#endif



