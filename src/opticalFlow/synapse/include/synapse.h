#ifndef SYNAPSE_H
#define SYNAPSE_H

#include <iostream>
#include <string>
#include <cmath>

//#include <yarp/sig/all.h>
//#include <yarp/math/Math.h>

#include <gsl/gsl_matrix.h>

#include "eLucasKanade.h"
#include "params.h"

//#define _DEBUG

class synapse
{
public:
	synapse(){};
	synapse(int, void**);//, void (&i_fn2call)(void**));
	~synapse();
	void operator=(const synapse&);
	inline void fnCall(int argc, void** argv){filter(argc, argv);};
	inline void openPort(std::string& str){objFlow.openPort(str);};
    inline void closePort(){objFlow.closePort();};
/**
 * void printArg() permit to print the parameter sent to the constructor of synapse, and call the method pointed
 */
//	void printArg();
private:
/**********
 * Methods
 *********/
/**
 * void filter(int argc, int* argv)
 * @param argc: the number of argument
 * @param argv: the pointer on the argument
 */
	void filter(int, void**);
	inline int* v2i(void* arg){return reinterpret_cast<int*>(arg);};
	void setArg();
	void exp(yarp::sig::Matrix&);
	void updateSubMatrix(yarp::sig::Matrix&, yarp::sig::Matrix&, int, int, int, int);
	int findSupEq(yarp::sig::Matrix&, double, yarp::sig::Matrix&);
/************
 * Variables
 ***********/

/**
 * generic parameter(s)
 */
	int sWin;
	int* sMapX;
	int* sMapY;
	int* neighLR;
	int* accTime;

/**
 * parameters of the synapse
 */
	int refTs;
    double stdDev2;
	double* stdDev;
	int* threshold; //Value the accumulation of windowed-spike have to reach to involve the call of the specified function
	int* alpha;
	int* tauC;
	int* tauD;

	yarp::sig::Matrix td;
	yarp::sig::Matrix potential;
	yarp::sig::Matrix discharge;
	yarp::sig::Matrix gaussian;
	yarp::sig::Matrix repT;

    yarp::sig::Matrix tmpSub;
    yarp::sig::Matrix tmpSub2;

    yarp::sig::Matrix zeroMat;

	yarp::sig::Vector X;
	yarp::sig::Vector Y;
/**
 * paramters for post computation
 */
	int* nbArg;
	int* selectedArg;
//	void (*fn2call)(int, void**);
//	void (eLucasKanade::*fn2call)(int, void**);
    void** argELK;
	eLucasKanade objFlow;
	void** arg;

	yarp::sig::Matrix gs;
	yarp::sig::Matrix pgs;
	yarp::sig::Matrix subGs;
};

#endif //SYNAPSE_H
