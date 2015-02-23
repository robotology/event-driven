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
 * @file vtsOptFlow.cpp
 * @brief Implementation of the vtsOptFlow (see header file).
 */

#include "vtsOptFlow.hpp"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace emorph;


bool vtsOptFlow::configure(ResourceFinder &rf)
{
    /* set the name of the module */
    moduleName            = rf.check("name",
                           Value("vtsOptFlow"),
                           "module name (string)").asString();

    setName(moduleName.c_str());

    /* open and attach rpc port */
    rpcPortName =  "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName)) {
        cerr << getName() << ": Unable to open rpc port at " << rpcPortName << endl;
        return false;
    }

    /* make the respond method of this RF module respond to the rpcPort */
    attach(rpcPort);

    /* set parameters */
    unsigned int height = rf.find("height").asInt();
    unsigned int width = rf.find("width").asInt();
    //unsigned int accumulation = rf.find("acc").asInt();
    unsigned int binAcc = rf.find("bin").asInt();
    double threshold = rf.find("threshold").asDouble();
    //unsigned int neighbor = rf.find("nNeighBor").asInt();
    unsigned int sobelSz = rf.find("szSobel").asInt();
    unsigned int tsVal = rf.find("tsVal").asInt();
    double alpha = rf.find("alpha").asDouble();
    //double tauD = rf.find("tauD").asDouble();
    int eye = rf.find("eye").asInt();
    bool saveOf = rf.check("save");
    bool orientation = rf.check("swap_xy");

    vtsofManager = new vtsOptFlowManager( moduleName, height, width,  binAcc, threshold, sobelSz, tsVal, alpha, eye, saveOf, orientation );
    vtsofManager->open();

    return true ;
}

bool vtsOptFlow::interruptModule()
{
    rpcPort.interrupt();
    vtsofManager->interrupt();

    return true;
}

bool vtsOptFlow::close()
{
    rpcPort.close();
    vtsofManager->close();
    delete vtsofManager;

    return true;
}

/* Called periodically every getPeriod() seconds */
bool vtsOptFlow::updateModule()
{
    return true;
}

vtsOptFlowManager::~vtsOptFlowManager()
{
    delete[] TSsData;
    delete[] TSs2PlanData;
    delete[] activityData;
    delete[] xNeighFlow;
    delete[] yNeighFlow;
    delete[] vxMatData;
    delete[] vyMatData;
    delete[] vxMean;
    delete[] vyMean;
    delete[] trans2neigh;
    delete vxMat;
    delete vyMat;
}

vtsOptFlowManager::vtsOptFlowManager(const std::string &moduleName, unsigned int &_height, unsigned int &_width, unsigned int &_binAcc, double &_threshold, unsigned int &_sobelSz, unsigned int &_tsVal, double &_alpha, int &_eye, bool &_saveOf, bool &_orientation)
    :activity(_height, _width), TSs(_height, _width), TSs2Plan(_height, _width), compPurpTS(_height, _width), sobelx(_sobelSz, _sobelSz), sobely(_sobelSz, _sobelSz), subTSs(_sobelSz, _sobelSz), A(_sobelSz*_sobelSz, 3), At(3,_sobelSz*_sobelSz), AtA(3,3), abc(3), Y(_sobelSz*_sobelSz), ctrl((uint)floor((double)_sobelSz/2.0), (uint)floor((double)_sobelSz/2.0)), sMat(_height*_width, 2), binEvts(10000, 3), alreadyComputedX(_height, _width), alreadyComputedY(_height, _width), binAcc(_binAcc)
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;

    TSsData=TSs.data();
    TSs2PlanData=TSs2Plan.data();
    activityData=activity.data();

    alpha=_alpha;
    threshold=_threshold;
    //tauD=_tauD;
    tsVal=_tsVal;
    //neighbor=_neighbor;
    //neighLR=(uint)floor((double)_neighbor/2.0);
    sobelSz=_sobelSz;
    sobelLR=(uint)floor((double)_sobelSz/2.0);
    height=_height;
    width=_width;
    //accumulation=_accumulation;
    saveOf=_saveOf;
    orientation=_orientation;

    vxMean=new double[_height*_width]; memset(vxMean, 0, _height*_width*sizeof(double));
    vyMean=new double[_height*_width]; memset(vyMean, 0, _height*_width*sizeof(double));
    ivxyNData=new uint[_height*_width]; memset(ivxyNData, 0, _height*_width*sizeof(uint));

    //xNeighFlow=new double[_nn*_nn];
    xNeighFlow=new double[_sobelSz*_sobelSz];
    //yNeighFlow=new double[_nn*_nn];
    yNeighFlow=new double[_sobelSz*_sobelSz];
    ixNeighFlow=0;
    iyNeighFlow=0;

    first=true;

    setSobelFilters(_sobelSz, sobelx, sobely);

    //polarity=0;
    activity=0;
    TSs=0;
    TSs2Plan=0;
    compPurpTS=0;
    ctrl=-1;

    vxMat = new Matrix(_height, _width);
    vyMat = new Matrix(_height, _width);

    *vxMat=0; vxMatData=vxMat->data();
    *vyMat=0; vyMatData=vyMat->data();

    sMat=0;
    iSMat=0;

    //borneInfX=neighLR+1+sobelLR;
    borneInfX=2*sobelLR+1;
    //borneInfY=neighLR+1+sobelLR;
    borneInfY=2*sobelLR+1;
    borneSupX=_height-borneInfX;
    borneSupY=_width-borneInfY;

    trans2neigh = new int[2*(4*_sobelSz-4)];
    iT2N=0;
    for(int i=0; i<_sobelSz; ++i)
        for(int ii=0; ii<_sobelSz; ++ii)
        {
            if(i==0 || i==(_sobelSz-1) || ii==0 || ii==(_sobelSz-1))
            {
                *(trans2neigh+iT2N*2)=i-(int)sobelLR;
                *(trans2neigh+iT2N*2+1)=ii-(int)sobelLR;
                ++iT2N;
            }
        }
    if(saveOf)
        saveFile.open("optical_flow_events.txt", ios::out);
}

bool vtsOptFlowManager::open()
{
    /*create all ports*/
    this->useCallback();

    inPortName = "/" + moduleName + "/vBottle:i";
    BufferedPort<emorph::vBottle>::open(inPortName);

    outPortName = "/" + moduleName + "/vBottle:o";
    outPort.open(outPortName);
    return true;
}

void vtsOptFlowManager::close()
{
    /*close ports*/
    this->close();
    outPort.close();

    delete[] TSsData;
    delete[] TSs2PlanData;
    delete[] activityData;
    delete[] xNeighFlow;
    delete[] yNeighFlow;
    delete[] vxMatData;
    delete[] vyMatData;
    delete[] vxMean;
    delete[] vyMean;
    delete[] trans2neigh;
    delete vxMat;
    delete vyMat;
}

void vtsOptFlowManager::interrupt()
{
    this->interrupt();
    outPort.interrupt();
}

void vtsOptFlowManager::onRead(emorph::vBottle &bot)
{
    uint refbin;
    uint prefbin;

    /*create event queue*/
    emorph::vQueue q;
    /*create queue iterator*/
    emorph::vQueue::iterator qi;

    /*prepare output vBottle with address events extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    emorph::OpticalFlowEvent outEvent;

    /*get the event queue in the vBottle bot*/
    bot.getAll(q);

    iBinEvts = 0;

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        posX = aep->getX();
        posY = aep->getY();
        ts = aep->getStamp();
        channel = aep->getChannel();

        /*correct wrap around*/
        ts = unwrapper(ts);

        /*swap xy*/
        if(orientation)
        {
            posXY = posY;
            posY = posX;
            posX = posXY;
        }

        /*compute only for one channel*/
        if (eye != channel) continue;

        /*fill the bin with events*/
        if(first)
        {
            first=false;
            refts=ts;
            TSs=ts;
            iBinEvts=0;
            refbin=ts;
            //prefbin=ts;
        }
        else
        {
            uint ii=0;
            refbin=((1-alpha)*ts+alpha*prefbin);

            for(uint i=0; i<iBinEvts;++i)
            {
                if(binEvts(i, 2)>=refbin)
                {
                    binEvts(ii, 0)=binEvts(i, 0);
                    binEvts(ii, 1)=binEvts(i, 1);
                    binEvts(ii, 2)=binEvts(i, 2);
                    ++ii;
                }
                else
                {
                    *(vxMean+(int)binEvts(i, 0)*width+(int)binEvts(i, 1))=0;
                    *(vyMean+(int)binEvts(i, 0)*width+(int)binEvts(i, 1))=0;
                    *(ivxyNData+(int)binEvts(i, 0)*width+(int)binEvts(i, 1))=0;
                }
            }
            iBinEvts=ii;
        }

        binEvts(iBinEvts, 0)=posX;
        binEvts(iBinEvts, 1)=posY;
        binEvts(iBinEvts, 2)=ts;
        iBinEvts++;

        ts=prefbin=refbin;

        if(iBinEvts)
        {
            /*update activity*/
            updateAll();

            /*compute the flow*/
            outEvent = compute();

            vx = outEvent.getVx();
            vy = outEvent.getVy();

            /*
           vx = outEvent.getVx();
           computeStat(vx,ixNeighFlow,mean,dev_std);
           mean=mean;
           dev_std=dev_std;
           cout << "mean " << mean << endl;
           cout << "std dev " << dev_std << endl;
           */

            /*add optical flow events to the out bottle only if the number of good computation was >= 3*/
            if (vx!=0 || vy!=0)
            {
                outBottle.addEvent(outEvent);
            }
        }
    }
    /*send on the processed events*/
    outPort.write();
}

void vtsOptFlowManager::updateAll()
{
    iSMat=0;    //number of addresses where the activity is sufficient
/*    for(uint i=0; i<height; ++i)
        for(uint ii=0; ii<width; ++ii)
        {
            if(*(TSsData+i*width+ii)+tsVal<ts)
            {
                *(TSs2PlanData+i*width+ii)=-1;
                *(activityData+i*width+ii)=0;
            }
        }*/

    for(uint i=0; i<iBinEvts; i++)
    {
      posX=binEvts(i, 0);
      posY=binEvts(i, 1);

       if(*(TSsData+posX*width+posY)+tsVal<ts)
           *(activityData+posX*width+posY)=0;
       *(TSsData+posX*width+posY)=(double)ts;
       *(TSs2PlanData+posX*width+posY)=(double)ts;

       /*update activity and use only pixels whose activity aboves the threshold to compute the flow */
        *(activityData+posX*width+posY)+=1;
        if(*(activityData+posX*width+posY)>=threshold)
        {
            sMat(iSMat, 0)=posX;
            sMat(iSMat, 1)=posY;
            iSMat++;
            //std::cout << "Threshold reached, flow to compute: " << iSMat << std::endl;
            //std::cout << "Current activity: " << *(activityData+posX*width+posY) << std::endl;
        }
    }
    //std::cout << "Number of addr were the activity is suffisant: " << iSMat << std::endl;
    //printMatrix(activity);
}

emorph::OpticalFlowEvent vtsOptFlowManager::compute()
{
    emorph::OpticalFlowEvent opt_flow;
    alreadyComputedX=0;
    alreadyComputedY=0;
    for(uint ixy=0; ixy<iSMat; ++ixy)
    {
        uint x=sMat(ixy,0);
        uint y=sMat(ixy,1);

        //std::cout << "[tsOptFlowThread] Current address: " << x << " " << y << " at: " << *(TSsData+x*width+y) << std::endl;
        if(x>borneInfX && x<borneSupX && y>borneInfY && y<borneSupY)
        {
            ixNeighFlow=0;
            iyNeighFlow=0;
            //for(uint xn=x-neighLR; xn<=x+neighLR; ++xn)
            for(int it=0; it<iT2N; ++it)
            {
                uint xn=x+*(trans2neigh+2*it);
                uint yn=y+*(trans2neigh+2*it+1);
                //std::cout << "[tsOptFlowThread] xn: " << xn << ", yn: " << yn << ", xn-sobelLR: " << xn-sobelLR << ", xn+sobelLR: " << xn+sobelLR << ", yn-sobelLR: " << yn-sobelLR << ", yn+sobelLR: " << yn+sobelLR << std::endl;

                //for(uint yn=y-neighLR; yn<=y+neighLR; ++yn)
                //{
                    if(alreadyComputedX(xn, yn) || alreadyComputedY(xn, yn))//if(alreadyComputedX(xn, yn)!=-1)
                    {
                        *(xNeighFlow+ixNeighFlow++)=alreadyComputedX(xn, yn);
                        *(yNeighFlow+iyNeighFlow++)=alreadyComputedY(xn, yn);
                    }
                    else
                    {
#ifdef _DEBUG
                        std::cout << "[tsOptFlowThread] Extract ts submat at (" << xn << ", " << yn << ")" << std::endl;
#endif

                        //subTSs=TSs.submatrix(xn-sobelLR, xn+sobelLR, yn-sobelLR, yn+sobelLR);
                        subTSs=TSs2Plan.submatrix(xn-sobelLR, xn+sobelLR, yn-sobelLR, yn+sobelLR);

#ifdef _DEBUG
                        cout << "subTSs = " << endl;
                        printMatrix(subTSs);
                        std::cout << "[tsOptFlowThread] Start create plan at (" << xn << ", " << yn << ")" << std::endl;
#endif
                        //uint resp=createPlan(subTSs);
                        dx=dy=0;
                        //uint resp=createPlanAndCompute(subTSs, dx, dy);
                        uint resp=createPlanAndCompute(subTSs, dx, dy, sobelLR, sobelLR, ts);
#ifdef _DEBUG
                        std::cout << "[tsOptFlowThread] Plan created at (" << xn << ", " << yn << ")" << std::endl;
                        //printMatrix(subTSs);
#endif
                        if(resp)
                        //if(createPlan(subTSs))
                        {
#ifdef _DEBUG
                            //std::cout << "[tsOptFlowThread] Plan computed" << std::endl;
                            std::cout << "[tsOptFlowThread] Compute flow (" << xn << ", " << yn << ")" << std::endl;
#endif
//#ifdef _ANALYSE_
//                           smoothedNeigh++;
//#endif
    /*                        dx=0;
                            dy=0;
                            for(uint i=0; i<neighbor; i++)
                            {
                                for(uint ii=0; ii<neighbor; ii++)
                                {
                                    dx+=sobelx(i, ii)*subTSs((neighbor-1)-i, (neighbor-1)-ii);
                                    dy+=sobely(i, ii)*subTSs((neighbor-1)-i, (neighbor-1)-ii);
                                    //TO BE CONTINUE
                                }
                            }*/
                            //alreadyComputedX(xn, yn)=dx;
                            //alreadyComputedY(xn, yn)=dy;
                            alreadyComputedX(xn, yn)=((dx/4)*1e-6)/3;
                            alreadyComputedY(xn, yn)=((dy/4)*1e-6)/3;
                            if(dx || dy)
                            {
                                //*(xNeighFlow+ixNeighFlow++)=dx;
                                //*(yNeighFlow+iyNeighFlow++)=dy;
                                *(xNeighFlow+ixNeighFlow++)=alreadyComputedX(xn, yn);//((dx/4)*1e-6)/3;
                                *(yNeighFlow+iyNeighFlow++)=alreadyComputedY(xn, yn);//((dy/4)*1e-6)/3;
                            }
#ifdef _DEBUG
                            std::cout << "[tsOptFlowThread] Flow (" << xn << ", " << yn << ") computed and stored" << std::endl;
#endif

                        }
                    }
                //}
            }
            //std::cout << "Number of good computation: " << ixNeighFlow << endl;
            if(ixNeighFlow>=3)
            {
#ifdef _ANALYSE_
                smoothedNeigh++;
#endif
/*
                gsl_sort(xNeighFlow, 1, ixNeighFlow);
                gsl_sort(yNeighFlow, 1, iyNeighFlow);
                double outX=gsl_stats_median_from_sorted_data(xNeighFlow, 1, ixNeighFlow);
                double outY=gsl_stats_median_from_sorted_data(yNeighFlow, 1, iyNeighFlow);
                //outX=((outX)*1E-9);
                //outY=((outY)*1E-9);
                //velBuf.addData((short)x, (short)y, outY, outX, ts);
*/
#ifdef _DEBUG
                //std::cout << "posX: " << x << ", posY: " << y << ", vx: " << *(vxMatData+x*height+y) << ", vy: " << *(vyMatData+x*height+y) << ", ts: " << ts << std::endl;
                //std::cout << "posX: " << x << ", posY: " << y << ", vx: " << medianX << ", vy: " << medianY << ", ts: " << ts << std::endl;
#endif

                double outX=gsl_stats_mean(xNeighFlow, 1, ixNeighFlow);
                double outY=gsl_stats_mean(yNeighFlow, 1, iyNeighFlow);

                //outX=((outX)*1E-9)/2;
                //outY=((outY)*1E-9)/2;

                //velBuf.addData((short)x, (short)y, outY, outX, ts);
                //std::cout << "Mutex free?" << std::endl;
                *(ivxyNData+x*width+y)+=1;
                *(vxMean+x*width+y)=*(vxMean+x*width+y)+(1/ *(ivxyNData+x*width+y))*(outX-*(vxMean+x*width+y));
                *(vyMean+x*width+y)=*(vyMean+x*width+y)+(1/ *(ivxyNData+x*width+y))*(outY-*(vyMean+x*width+y));

                opt_flow.setX(x);
                opt_flow.setY(y);
                opt_flow.setVx(float(*(vxMean+x*width+y)));
                opt_flow.setVy(float(*(vyMean+x*width+y)));
                opt_flow.setStamp(ts);

                if(saveOf)
                {
                    line2save.str("");
                    line2save << (short)x << " " << (short)y << " " << *(vxMean+x*width+y) << " " << *(vyMean+x*width+y) << " " << ts << endl;
                    saveFile.write( line2save.str().c_str(), line2save.str().size() );
                }
                else
                {
                    *(vxMatData+x*height+y)=0;
                    *(vyMatData+x*height+y)=0;
                }
                return opt_flow;

            }

        }
    }
}


uint vtsOptFlowManager::createPlanAndCompute(Matrix &_m, double &_dx, double &_dy, uint &_curx, uint &_cury, uint &_curts)
{
#ifdef _DEBUG
    std::cout << "[tsOptFlowThread] Create the plan" << std::endl;
#endif
    uint test=(_m.submatrix(0, sobelLR-1, 0, sobelLR-1)==ctrl)?0:1;
    test+=(_m.submatrix(0, sobelLR-1, sobelLR+1, sobelSz-1)==ctrl)?0:1;
    test+=(_m.submatrix(sobelLR+1, sobelSz-1, 0, sobelLR-1)==ctrl)?0:1;
    test+=(_m.submatrix(sobelLR+1, sobelSz-1, sobelLR+1, sobelSz-1)==ctrl)?0:1;
    //printMatrix(_m);

    if(test>=3)
    {
#ifdef _DEBUG
    std::cout << "[tsOptFlowThread] Condition passed" << std::endl;
#endif
        int index=-1;
        uint i;
        uint ii;
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Cover the submatrix" << std::endl;
#endif  
        A.resize(sobelSz*sobelSz, 3);
        Y.resize(sobelSz*sobelSz);
        for(i=0; i<sobelSz; i++)
            for(ii=0; ii<sobelSz; ii++)
            {
                //if(_m(i, ii)>-1 && ((i==_curx && ii==_cury) || ((i!=_curx || ii!=_cury) && _m(i, ii)<=(_curts-binAcc))))
                if( (_m(i, ii)+tsVal)>_curts && ((i==_curx && ii==_cury) || ((i!=_curx || ii!=_cury) && _m(i, ii)<=(_curts-binAcc))))
                {
#ifdef _DEBUG
                    std::cout << "[tsOptFlowThread] Following index: " << index+1 << ", size A: [" << A.rows() << ", " << A.cols() << "], size Y: " << Y.size() << std::endl;
#endif
                    A(++index, 0)=i;
                    A(index, 1)=ii;
                    A(index, 2)=1;
                    Y(index)=_m(i, ii);
                }
            }

        if(index<3)
            return 0;
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Compute the LS-plan" << std::endl;
#endif
        A=A.submatrix(0, index, 0, 2);//A.resize(index, 3);
        Y=Y.subVector(0, index); //Y.resize(index);
        At=A.transposed();
        AtA=At*A;
        //if(AtA.rows()==3)
        //{
/*DET = a11(a33a22-a32a23)-a21(a33a12-a32a13)+a31(a23a12-a22a13)*/
/*
| a11 a12 a13 |-1             |   a33a22-a32a23  -(a33a12-a32a13)   a23a12-a22a13  |
| a21 a22 a23 |    =  1/DET * | -(a33a21-a31a23)   a33a11-a31a13  -(a23a11-a21a13) |
| a31 a32 a33 |               |   a32a21-a31a22  -(a32a11-a31a12)   a22a11-a21a12  |
*/          double* dataATA=AtA.data();
            double DET=*dataATA*( *(dataATA+8)**(dataATA+4)-*(dataATA+7)**(dataATA+5)) - *(dataATA+3)*(*(dataATA+8)**(dataATA+1)-*(dataATA+7)**(dataATA+2))+*(dataATA+6)*(*(dataATA+5)**(dataATA+1)-*(dataATA+4)**(dataATA+2));
            if(!DET)
                return 0;
            A.resize(3,3);
            //cout << "A = " << endl;
            //printMatrix(A);
            double *dataA=A.data();
            DET=1/DET;
            *dataA=DET*(*(dataATA+8)**(dataATA+4)-*(dataATA+7)**(dataATA+5));
            *(dataA+1)=DET*(*(dataATA+7)**(dataATA+2)-*(dataATA+8)**(dataATA+1));
            *(dataA+2)=DET*(*(dataATA+5)**(dataATA+1)-*(dataATA+4)**(dataATA+2));
            *(dataA+3)=DET*(*(dataATA+6)**(dataATA+5)-*(dataATA+8)**(dataATA+3));
            *(dataA+4)=DET*(*(dataATA+8)**dataATA-*(dataATA+6)**(dataATA+2));
            *(dataA+5)=DET*(*(dataATA+3)**(dataATA+2)-*(dataATA+5)**dataATA);
            *(dataA+6)=DET*(*(dataATA+7)**(dataATA+3)-*(dataATA+6)**(dataATA+4));
            *(dataA+7)=DET*(*(dataATA+6)**(dataATA+1)-*(dataATA+7)**dataATA);
            *(dataA+8)=DET*(*(dataATA+4)**dataATA-*(dataATA+3)**(dataATA+1));
            //cout << "At = " << endl;
            //printMatrix(At);
            //cout << "Y = " << endl;
            //for(uint iii=0; iii<index; iii++)
            //    cout << Y(i) << endl;
            abc=A*At*Y;
            //cout << "abc = " << abc(0) << " " << abc(1) << " " << abc(2) << endl;
        //}
        //else
        //    abc=pinv(AtA)*At*Y;
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Fill the plan with the new values" << std::endl;
#endif
        for(i=0; i<sobelSz; i++)
            for(ii=0; ii<sobelSz;ii++)
            {
                //_m(i, ii)=abc(0)*i+abc(1)*ii+abc(2);
                dx+=sobelx(i,ii)*(abc(0)*((sobelSz-1)-i)+abc(1)*((sobelSz-1)-ii)+abc(2));
                dy+=sobely(i,ii)*(abc(0)*((sobelSz-1)-i)+abc(1)*((sobelSz-1)-ii)+abc(2));
            }
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Plan filled" << std::endl;
#endif
        return 1;
    }
    else
    {
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] return 0" << std::endl;
#endif
        return 0;
    }
}

void vtsOptFlowManager::setSobelFilters(uint _sz, Matrix& _sfx, Matrix& _sfy)
{
    Vector Sx(_sz);
    Vector Dx(_sz);
    for(int i=1; i<=_sz; i++)
    {
        //std::cout << "\t" << i << std::endl;
        //std::cout << "\t" << _sz-1 << "! = " << factorial(_sz-1) << std::endl;
        Sx(i-1)=factorial((_sz-1))/((factorial((_sz-1)-(i-1)))*(factorial(i-1)));
        Dx(i-1)=Pasc(i-1,_sz-2)-Pasc(i-2,_sz-2);
    }
    //Sy=Sx';
    //Dy=Dx';
    _sfx=outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
    _sfy=_sfx.transposed();
}

int vtsOptFlowManager::factorial(int _v)
{
    if(_v<=1)
        return 1;
    return _v*factorial(_v-1);
}

int vtsOptFlowManager::Pasc(int k, int n)
{
    int P;
    if ( (k>=0) && (k<=n) )
        P=factorial(n)/(factorial(n-k)*factorial(k));
    else
        P=0;
    return P;
}

void vtsOptFlowManager::printMatrix(Matrix& _mat)
{
    uint nr=_mat.rows();
    uint nc=_mat.cols();
    for(uint r=0; r<nr; r++)
    {
        for(uint c=0; c<nc; c++)
            std::cout << _mat(r,c) << " ";
        std::cout << std::endl;
    }
}

/*
int vtsOptFlowManager::computeStat(double _v, uint _dim, double &_mean, double &_std)
{
    double m1, m2;

    m1=0;
    m2=0;

    for(int i=0; i<_dim; i++)
    {
        m1=m1+_v[i];
        m2=m2+_v[i]*_v[i];
    }
    m1=m1/_dim;
    m2=m2/_dim;

    mean=m1;
    std=sqrt(m2-m1*m1);

    return 0;
}
*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------
