#ifndef ELUCASKANADE_H
#define ELUCASKANADE_H

#include <iostream>
#include <string>
#include <cmath>

#include <yarp/os/all.h>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>

#include "vecBuffer.h"

class eLucasKanade
{
public:
	eLucasKanade(){};
	eLucasKanade(int, void**);
	~eLucasKanade();
    void operator=(const eLucasKanade&);

	inline void fnCall(int argc, void** argv){compute(argc, argv);};
    inline void openPort(std::string& str){port.open(str.c_str());};
    inline void closePort(){port.close();};
private:
/*********
 * Methods
 * *******/
	void compute(int, void**);
	void send(int&, int&, double&, double&);

	yarp::sig::Matrix* v2m(void* arg){return reinterpret_cast<yarp::sig::Matrix*>(arg);};
	yarp::sig::Vector* v2uvec(void* arg){return reinterpret_cast<yarp::sig::Vector*>(arg);};
	void eig(yarp::sig::Matrix&, yarp::sig::Vector&, yarp::sig::Matrix&);
	void eigNonSym(yarp::sig::Matrix& in, yarp::sig::Vector& eval, yarp::sig::Matrix& evec);
	void exp(yarp::sig::Matrix&);
	yarp::sig::Vector mat2vec(yarp::sig::Matrix&);
/************
 * Variables
 * *********/
    int lambda;
	int sWin;

	int* sMapX;
	int* sMapY;
	int* neighLR;
	int* stdDev;

	yarp::sig::Matrix gaussian;
	yarp::sig::Matrix W;
    yarp::sig::Matrix W_s;

	yarp::sig::Matrix sobelX;
	yarp::sig::Matrix sobelY;

	yarp::sig::Matrix dIdxy;

	yarp::sig::Matrix neighboorhood;

//	arma::colvec dIdx;
//	arma::colvec dIdy;
	yarp::sig::Vector dIdt;
    yarp::sig::Vector zeroVec;

    yarp::os::BufferedPort<vecBuffer> port;
};

#endif //ELUCASKANADE_H
