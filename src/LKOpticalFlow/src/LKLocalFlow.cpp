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


#include <iCub/LKLocalFlow.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Math.h>

POLARITY_TYPE SOBEL_X[3][3] = {{-1, 0, 1} ,{-2, 0, 2}, {-1, 0, 1}};
POLARITY_TYPE SOBEL_Y[3][3] = {{-1, -2, -1} ,{0, 0, 0}, {1, 2, 1}};

LKLocalFlow::LKLocalFlow(int nRadius)
{
	GuaWeight = NULL;
	lambda = 1.5;
	neighborRadius = nRadius;
	windowLength = 2 * nRadius + 1;
}


void LKLocalFlow::calVelocity(/*TIMESTAMP_TYPE*/double  tsDiff, MyMatrix<TIMESTAMP_TYPE> * ts, MyMatrix<POLARITY_TYPE>* It, MyMatrix<POLARITY_TYPE> * Ipt, int rowIdx, int clmnIdx, double * velocity){
	POLARITY_TYPE * tmpDblPtr;

    MyMatrix<double> A(windowLength*windowLength, 2);
    MyMatrix<double> b(windowLength*windowLength, 1);

    //TODO invalid access for the points in the border

    for (int rCntr = 0; rCntr < windowLength; ++rCntr) {
        for (int cCntr = 0; cCntr < windowLength; ++cCntr) {

            // Apply Sobel Filter to calculate Ix and Iy
            tmpDblPtr =  *SOBEL_X;

            A(rCntr*windowLength + cCntr, 0) = It->convolveSubMatrix(rowIdx - neighborRadius - 1 + rCntr,
                                                       clmnIdx - neighborRadius - 1 + cCntr,
                                                       tmpDblPtr, 3,3);        //calculate Ix


            tmpDblPtr = *SOBEL_Y;
            A(rCntr*windowLength + cCntr, 1) = It->convolveSubMatrix(rowIdx - neighborRadius - 1 + rCntr,
                                                       clmnIdx - neighborRadius - 1 + cCntr,
                                                       tmpDblPtr, 3,3);          // calculate Iy
            b(rCntr*windowLength + cCntr,0) = Ipt-> operator ()(rowIdx -neighborRadius + rCntr,
                                                clmnIdx -neighborRadius + cCntr ) -
                                             It->operator ()(rowIdx -neighborRadius + rCntr,
                                                 clmnIdx -neighborRadius + cCntr ); // calculate It


///           b(rCntr*windowLength + cCntr,0) = b(rCntr*windowLength + cCntr,0) / ( tsDiff /*+ .00001*/) ;
        } // end for on columns
    }//end for on rows


    LeastSquareSolver(A, b, velocity);


/*if (*velocity < 0){
    It->printSubMatrix(rowIdx - rNeighborRadius - 1, clmnIdx - cNeighborRadius -1, rWindow + 2, cWindow + 2);
    b.printSubMatrix(0,0,25,1);
    cout << "---------- " << rowIdx << " " << clmnIdx << " " << *velocity << " " << *(velocity + 1) << endl;
    cout << "-----------------------------------------------------------" << endl;
}*/

}


void LKLocalFlow::LeastSquareSolver(MyMatrix<double> A, MyMatrix<double> b, double * velocity){
    short ARows;
    double mtxDet, eigenValMax, eigenValMin, tmp;
    double eigVec [2];
    double wIx2 = 0;
    double wIy2 = 0;
    double wIxy = 0;
    double w;
    double wIxB =0;
    double wIyB = 0;

    ARows = windowLength * windowLength;
    for (int i = 0; i < ARows; ++i) {
        w = *(GuaWeight + i);
        wIx2 += w * A(i,0) * A(i,0);

        wIy2 += w * A(i,1) * A(i,1);
        wIxy += w * A(i,0) * A(i,1);

        wIxB += w * A(i,0)*b(i,0);
        wIyB += w * A(i,1)*b(i,0);
    }

    /*
     wIx2  wIxy
     wIxy  wIy2 */
    mtxDet = wIx2*wIy2 - wIxy*wIxy;

//  cout << wIx2 << " " << wIy2 << " " << wIxy << " " << wIxB <<" " << wIyB << " det " << mtxDet << endl;

    //The solution
    yarp::sig::Matrix a(2,2);
    yarp::sig::Matrix inva(2,2);
    a(0,0) = wIx2;
    a(0,1) = wIxy;
    a(1,0) = wIxy;
    a(1,1) = wIy2;
    inva = yarp::math::pinv(a);
    * velocity = inva(0,0)*wIxB + inva(0,1)*wIyB;
    * (velocity + 1) = inva(1,0)*wIxB + inva(1,1)*wIyB;

 //   cout << "Vx : " << *velocity << " Vy : " << *(velocity + 1) << endl;

    //Two EigenValues
    tmp = sqrt((wIx2 - wIy2)*(wIx2 - wIy2) + 4 * wIxy* wIxy); // sqrt(b^2 - 4ac
    eigenValMax = 0.5 * (wIx2 + wIy2 + tmp);
    eigenValMin = 0.5 * (wIx2 + wIy2 - tmp);

 //   cout << "eV Max : " << eigenValMax << "  eV Min: " << eigenValMin << endl;

    //Error checking
    if (eigenValMax < lambda){
        * velocity = 0;
        * (velocity + 1) = 0;
//        cout << "both eighenvalue are small" << endl;
    }else{
        if (eigenValMin < lambda){
           //EigenVector corresponding to the largest EigenValue
           if (wIxy != 0){
              tmp = 1 / sqrt(wIxy * wIxy + (eigenValMax - wIx2)*(eigenValMax - wIx2));
              *eigVec = wIxy * tmp; // b / (sqrt (b^2 + (eigValue - a)^2) )
              *(eigVec + 1) = (eigenValMax - wIx2) * tmp; // (eigValue - a) / (sqrt (b^2 + (eigValue - a)^2) )
           }
           else{
               if (wIx2 > wIy2){
                   *eigVec = 1;
                   *(eigVec + 1) = 0;
               }else{
                   *eigVec = 0;
                   *(eigVec + 1) = 1;
               }
           }

 //          cout << "one eighenvalue is small " << *velocity << " " << *(velocity  + 1) << " eig Vector: " <<  *eigVec << " " << *(eigVec + 1)  << endl;
           // update the solution
           tmp = *eigVec * *velocity + *(eigVec + 1) * *(velocity + 1);
           *velocity = tmp * *eigVec;
           *(velocity + 1) = tmp * * (eigVec +1);

        }
    }

    *(velocity + 2) = eigenValMin;

}

void LKLocalFlow::pinv_symm2x2(double a11, double a12 ,double a22, MyMatrix<double> & invA){
    double dblTmp;
    double aTrnspa11, aTrnspa12, aTrnspa22;
    double eigValMax, eigValMin;
    double rEigVecMax[2], rEigVecMin[2];
    double singValMax, singValMin;
    double lEigVecMax[2], lEigVecMin[2];
    double inva11, inva12, inva21 , inva22;


    //generat A*transpose(A)
    aTrnspa11 = a11*a11 + a12*a12;
    aTrnspa12 = a12* (a11 + a22);
    aTrnspa22 = a12*a12 + a22*a22;

    //eigenValues of A*transpose(A)
    dblTmp = sqrt( (aTrnspa11 - aTrnspa22) * (aTrnspa11 - aTrnspa22) + 4 * aTrnspa12*aTrnspa12);
    eigValMax = .5 * (aTrnspa11 + aTrnspa22 + dblTmp);
    eigValMin = .5* (aTrnspa11 + aTrnspa22 - dblTmp);

    //eigenVectors
    eigenVector_symm2x2(aTrnspa11, aTrnspa12, aTrnspa22, eigValMax, rEigVecMax);
    eigenVector_symm2x2(aTrnspa11, aTrnspa12, aTrnspa22, eigValMin, rEigVecMin);

 //   cout << "right Eigen Vec " << *rEigVecMax << " " << *(rEigVecMax + 1) << " -- " << *rEigVecMin << " " <<*(rEigVecMin + 1) << endl;

    //singular values of A
    singValMax = sqrt(eigValMax);
    singValMin = sqrt(eigValMin);

    cout << "Singular Values " << singValMax << " " << singValMin << endl;


    singValMax = 1/singValMax;
    singValMin = 1/singValMin;
    //TODO if they are too small put them equal to zero

    //constructing the left matrix
    *lEigVecMax = (a11 * *rEigVecMax + a12* *(rEigVecMax + 1)) * singValMax;
    *(lEigVecMax + 1) = (a12 * *rEigVecMax + a22* *(rEigVecMax + 1))*singValMax;

    *lEigVecMin = (a11 * *rEigVecMin + a12* *(rEigVecMin + 1))*singValMin;
    *(lEigVecMin + 1) = (a12 * *rEigVecMin + a22* *(rEigVecMin + 1))*singValMin;

//    cout << "right Eigen Vec " << *lEigVecMax << " " << *(lEigVecMax + 1) << " -- " << *lEigVecMin << " " <<*(lEigVecMin + 1) << endl;



    inva11 = *lEigVecMax * sqrt(eigValMax);
    inva21 =  *(lEigVecMax+1) * sqrt(eigValMax);

    inva21 = *lEigVecMin * sqrt(eigValMin);
    inva22 =  *(lEigVecMin+1) * sqrt(eigValMin) ;

    dblTmp = inva11 * *rEigVecMax + inva12 **rEigVecMin;
    inva12 = inva11 * *(rEigVecMax+1) + inva12 **(rEigVecMin + 1);
    inva11 = dblTmp;
    dblTmp = inva21 * *rEigVecMax + inva22 **rEigVecMin;
    inva22 = inva21 * *(rEigVecMax+1) + inva22 **(rEigVecMin + 1);
    inva21 = dblTmp;

   // cout << "hmm {" << inva11 << ", " << inva12 << " } { "  << inva21 << " , " << inva22 << "}" << endl;


    //inverse matrix = rEigenValue * singular_Values  8 lEigenValue'
    inva11 = *rEigVecMax * singValMax;
    inva21 =  *(rEigVecMax+1) * singValMax;

    inva21 = *rEigVecMin * singValMin;
    inva22 =  *(rEigVecMin+1) * singValMin;

    dblTmp = inva11 * *lEigVecMax + inva12 **lEigVecMin;
    inva12 = inva11 * *(lEigVecMax+1) + inva12 **(lEigVecMin + 1);
    inva11 = dblTmp;
    dblTmp = inva21 * *lEigVecMax + inva22 **lEigVecMin;
    inva22 = inva21 * *(lEigVecMax+1) + inva22 **(lEigVecMin + 1);
    inva21 = dblTmp;

    invA(0,0) = inva11;
    invA(0,1) = inva12;
    invA(1,0) = inva21;
    invA(1,1) = inva22;
}

void LKLocalFlow::eigenVector_symm2x2(double a11, double a12, double a22, double eigValue, double * eigVector){
    double tmp;
    if (a12 != 0){
        tmp = 1 / sqrt(a12*a12 + (eigValue - a22)*(eigValue - a22) );
        *eigVector = tmp * (eigValue - a22);
        *(eigVector+1) = tmp * a12;
    }
    else{
        if (a11 == eigValue){
            *eigVector = 1;
            *(eigVector + 1) = 0;
        }
        else {
            *eigVector = 0;
            *(eigVector + 1) = 1;
        }
    }
}



void LKLocalFlow::setGuaWeights(double stdDev){
    double * tmpPtr;

    if (GuaWeight == NULL){
        GuaWeight = new double [windowLength*windowLength];
    }


//    for (int i = 0; i < windowLength*windowLength; ++i) {
//        *(GuaWeight + i) = 1;
//    }
//
//    return;


    double tmp, tmp2;
    double* rowTmp = new double [windowLength];
    for (int idx = 0; idx < windowLength; ++idx) {
        *(rowTmp + idx) = (idx - neighborRadius)*(idx - neighborRadius);
    }

    tmpPtr = GuaWeight;
    for (int i = 0; i < windowLength; ++i) {
        tmp = *(rowTmp + i);
        for (int j = 0; j < windowLength; ++j) {
             *tmpPtr = *(rowTmp + j) + tmp;
             tmpPtr++;
        }
    }

    tmp = 2 * stdDev * stdDev;
    tmp2 = tmp;
    tmp = - 1 / tmp;
    tmp2 = 1 / ( tmp2 * 3.14);
    tmpPtr = GuaWeight;
    for (int i = 0; i < windowLength; ++i) {
        for (int j = 0; j < windowLength; ++j) {
            *tmpPtr = tmp2 * exp(*tmpPtr * tmp);
            tmpPtr++;
        }
    }
	delete [] rowTmp;
}

LKLocalFlow::~LKLocalFlow(){
    if (GuaWeight != NULL){
        delete [] GuaWeight;
    }

    cout << "LK local flow is closed finely" << endl;
}
