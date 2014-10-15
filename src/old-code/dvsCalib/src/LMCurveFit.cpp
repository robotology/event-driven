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

#include <iCub/LMCurveFit.h>
    
#ifndef FUNCTIONS_LM_MINIMIZATION
#define FUNCTIONS_LM_MINIMIZATION
int universalCounter =0;
int getFunction (const gsl_vector * x, void *data, gsl_vector * f){
       size_t n = ((struct data *)data)->n;
       double *y = ((struct data *)data)->y;
       double *sigma = ((struct data *) data)->sigma;
     
       double A = gsl_vector_get (x, 0);
       double lambda = gsl_vector_get (x, 1);
       double b = gsl_vector_get (x, 2);
     
       size_t i;
       printf("UniversalCount %d A%f lambda%f b%f\n",universalCounter++,A,lambda,b);
       
     
       for (i = 0; i < n; i++)
         {
           /* Model Yi = A * exp(-lambda * i) + b */
           double t = i;
           double Yi = A * exp (-lambda * t) + b;
           printf("%f, \t",Yi);
           gsl_vector_set (f, i, (Yi - y[i])/sigma[i]);
         }
     
       return GSL_SUCCESS;
}
     
int getDerivativeOfFunction (const gsl_vector * x, void *data,gsl_matrix * J){
       size_t n = ((struct data *)data)->n;
       double *sigma = ((struct data *) data)->sigma;
     
       double A = gsl_vector_get (x, 0);
       double lambda = gsl_vector_get (x, 1);
     
       size_t i;
     
       for (i = 0; i < n; i++)
         {
           /* Jacobian matrix J(i,j) = dfi / dxj, */
           /* where fi = (Yi - yi)/sigma[i],      */
           /*       Yi = A * exp(-lambda * i) + b  */
           /* and the xj are the parameters (A,lambda,b) */
           double t = i;
           double s = sigma[i];
           double e = exp(-lambda * t);
           gsl_matrix_set (J, i, 0, e/s); 
           gsl_matrix_set (J, i, 1, -t * A * e/s);
           gsl_matrix_set (J, i, 2, 1/s);
         }
       return GSL_SUCCESS;
}
   
int getFunctionAndDerivative (const gsl_vector * x, void *data,gsl_vector * f, gsl_matrix * J){
       getFunction (x, data, f);
       getDerivativeOfFunction (x, data, J);
     
       return GSL_SUCCESS;
}      	
  
        
#endif 
    
LMCurveFit::LMCurveFit(){

    //default ctor
}

LMCurveFit::LMCurveFit(int dim, int pts, double* yVal, double* sigVal){

    f.f = &getFunction;
    f.df = &getDerivativeOfFunction;
    f.fdf = &getFunctionAndDerivative;
    f.n = pts;
    f.p = dim;
    f.params = &dataStruct; 
    
    this->nbrOfDataPoints = pts;
    this->dimensionOfData = dim;
    this->Y = yVal;
    this->sigma = sigVal;
    dataStruct.n = pts;
    dataStruct.y = yVal;
    dataStruct.sigma = sigVal;
    
    
    
    covar = gsl_matrix_alloc (this->dimensionOfData, this->dimensionOfData);
    double x_init[3] = { 1.0, 0.0, 0.0 };
    x = gsl_vector_view_array (x_init, this->dimensionOfData);
    gsl_rng_env_setup();     
   type = gsl_rng_default;
   r = gsl_rng_alloc (type);
   T = gsl_multifit_fdfsolver_lmsder;
   s = gsl_multifit_fdfsolver_alloc (T, this->nbrOfDataPoints, this->dimensionOfData);  
   iterations = 0;
   gsl_multifit_fdfsolver_set (s, &f, &x.vector);
    
   
   
   
}
LMCurveFit::~LMCurveFit(){

    gsl_matrix_free (covar);
    gsl_rng_free (r);
    gsl_multifit_fdfsolver_free (s);

}

void LMCurveFit::initializeLM(){
  
   //f.f = boost::bind(&LMCurveFit::getFunction,this,_1,_2,_3);
   /*f.f = getFunction;
   f.df = getDerivativeOfFunction;
   f.fdf = getFunctionAndDerivative;
   f.n = this->nbrOfDataPoints;
   f.p = this->dimensionOfData;
   f.params = &dataStruct;  */  
     
    
} 
/*
LMCurveFit::release(){

    gsl_matrix_free (covar);
    gsl_rng_free (r);
       
}   
*/  		
       	
     	
     
     
     
 
     //void print_state (size_t iter, gsl_multifit_fdfsolver * s);
     
int LMCurveFit::solveLM (void){     
     
           
       
       
     
       //print_state (iter, s);
     
       do
         {
           iterations++;
           status = gsl_multifit_fdfsolver_iterate (s);
            print_state (iterations, s);
           printf ("status = %s\n", gsl_strerror (status));
     
           //print_state (iter, s);
     
           if (status)
             break;
     
           status = gsl_multifit_test_delta (s->dx, s->x,
                                             1e-4, 1e-4);
         }
       while (status == GSL_CONTINUE && iterations < LM_MAX_ITERATIONS);
     
       gsl_multifit_covar (s->J, 0.0, covar);
     
     #define FIT(i) gsl_vector_get(s->x, i)
     #define ERR(i) sqrt(gsl_matrix_get(covar,i,i))
     
       { 
         double chi = gsl_blas_dnrm2(s->f);
         double dof = this->nbrOfDataPoints - this->dimensionOfData;
         double c = GSL_MAX_DBL(1, chi / sqrt(dof)); 
     
         printf("chisq/dof = %g\n",  pow(chi, 2.0) / dof);
     
         printf ("A      = %.5f +/- %.5f\n", FIT(0), c*ERR(0));
         printf ("lambda = %.5f +/- %.5f\n", FIT(1), c*ERR(1));
         printf ("b      = %.5f +/- %.5f\n", FIT(2), c*ERR(2));
       }
     
       printf ("status = %s\n", gsl_strerror (status));
     
       
       
       return 0;
     }
     
void LMCurveFit::print_state (size_t iter, gsl_multifit_fdfsolver * s){
   printf ("iter: %3u x = % 15.8f % 15.8f % 15.8f "
           "|f(x)| = %g\n",
           iter,
           gsl_vector_get (s->x, 0), 
           gsl_vector_get (s->x, 1),
           gsl_vector_get (s->x, 2), 
           gsl_blas_dnrm2 (s->f));
 }
