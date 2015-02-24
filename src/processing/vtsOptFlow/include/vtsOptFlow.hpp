/*
 * Copyright (C) 2014 Istituto Italiano di Tecnologia
 * Author: Charles Clercq, edited by Valentina Vasco (01/15)
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

/*
 * @file tsOptFlow.h
 * @brief A module that reads vBottle from a yarp port and computes optical flow
 */
#ifndef VTSOPTFLOW_HPP
#define VTSOPTFLOW_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <cmath>

#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>

//#define _DEBUG

class vtsOptFlowManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    std::string moduleName;

    /*input and output port for the vBottle*/
    std::string outPortName;
    yarp::os::BufferedPort<emorph::vBottle> outPort;
    std::string inPortName;

    void setSobelFilters(uint, yarp::sig::Matrix&, yarp::sig::Matrix&);
    int factorial(int);
    int Pasc(int, int);
    void updateAll();
    emorph::OpticalFlowEvent compute();
    uint createPlan(yarp::sig::Matrix& );
    uint createPlanAndCompute(yarp::sig::Matrix&, double&, double&, uint&, uint&, uint&);
    void printMatrix(yarp::sig::Matrix& );
    //int computeStat(double, uint, double &, double &);

    /******************************************************************************/
    //   VARIABLES
    /******************************************************************************/

    double alpha;
    double threshold;
    //double tauD;
    uint tsVal;
    //uint neighbor;
    //uint neighLR;
    uint height;
    uint width;
    uint sobelSz;
    uint sobelLR;
    //uint accumulation;
    uint binAcc;
    bool saveOf;
    bool orientation;

    //double mean;
    //double dev_std;

    uint borneSupX;
    uint borneSupY;
    uint borneInfX;
    uint borneInfY;

    yarp::sig::Matrix sobelx;
    yarp::sig::Matrix sobely;

    emorph::vtsHelper unwrapper;

    yarp::sig::Matrix activity; double* activityData;
    yarp::sig::Matrix TSs;      double* TSsData;
    yarp::sig::Matrix TSs2Plan; double* TSs2PlanData;
    yarp::sig::Matrix compPurpTS;
    yarp::sig::Matrix subTSs;
    yarp::sig::Matrix alreadyComputedX;
    yarp::sig::Matrix alreadyComputedY;

    double  dx;
    double  dy;

    /*** To compute the plan ***/
    yarp::sig::Matrix A;
    yarp::sig::Matrix At;
    yarp::sig::Matrix AtA;
    yarp::sig::Matrix ctrl;
    yarp::sig::Vector abc;
    yarp::sig::Vector Y;
    /***************************/

    uint posX;
    uint posY;
    uint posXY;
    int channel;
    int pol;
    uint ts;
    uint eye;
    uint refts;
    bool first;

    yarp::sig::Matrix *vxMat;
    yarp::sig::Matrix *vyMat;
    double *vxMatData;
    double *vyMatData;

    yarp::sig::Matrix binEvts;
    uint iBinEvts;
    double *vxMean;
    double *vyMean;
    uint *ivxyNData;

    //yarp::sig::Matrix wMat;
    yarp::sig::Matrix sMat;
    uint iSMat;
    double *xNeighFlow;
    double *yNeighFlow;
    uint ixNeighFlow;
    uint iyNeighFlow;

    double vx;
    double vy;

    int *trans2neigh;
    int iT2N;

    std::ofstream saveFile;
    std::stringstream line2save;

public:

    vtsOptFlowManager(const std::string &moduleName, unsigned int &_height, unsigned int &_width, unsigned int &_binAcc, double &_threshold, unsigned int &_sobelSz, unsigned int &_tsVal, double &_alpha, int &_eye, bool &_saveOf, bool &_orientation);
    ~vtsOptFlowManager();

    bool    open();
    void    close();
    void    interrupt();

    /*function that reads vBottles*/
    void    onRead(emorph::vBottle &bot);

};


class vtsOptFlow:public yarp::os::RFModule {

    /* module parameters*/
    std::string moduleName;                     //name of the module (rootname of ports)
    std::string rpcPortName;                //name of the handler port (comunication with respond function)
    yarp::os::RpcServer rpcPort;            //a port to handle messages

    /* pointer to threads*/
    vtsOptFlowManager *vtsofManager;                //vtsOptFlowManager for processing vBottles and computing optical flow

public:

    virtual bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    virtual bool interruptModule();                       // interrupt, e.g., the ports
    virtual bool close();                                 // close and shut down the module
    virtual bool updateModule();
};
#endif //VTSOPTFLOW_HPP

//----- end-of-file --- ( next line intentionally left blank ) ------------------
