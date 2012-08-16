/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
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
#include "tsOptFlowThread.hpp"

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace emorph::eunmask;

tsOptFlowThread::tsOptFlowThread(uint &_h, uint &_w, std::string &_src, uint &_type, uint &_acc, uint &_bin, double &_th, uint &_nn, uint &_ssz, uint &_tsval, double &_a, double &_td, int &_pol, uint &_ori, bool &_save, Matrix *_vxMat, Matrix *_vyMat, Semaphore *_mutex, VelocityBuffer* _velb)
//:RateThread(THRATE), activity(_h, _w), TSs(_h, _w), TSs2Plan(_h, _w), compPurpTS(_h, _w), sobelx(_ssz, _ssz), sobely(_ssz, _ssz), subTSs(_nn, _nn), A(_nn*_nn, 3), At(3,_nn*_nn), AtA(3,3), abc(3), Y(_nn*_nn), ctrl((uint)floor((double)_ssz/2.0), (uint)floor((double)_ssz/2.0)), wMat(_h, _w), sMat(_h*_w, 2), binEvts(10000, 2), alreadyComputedX(_h, _w), alreadyComputedY(_h, _w)
:activity(_h, _w), TSs(_h, _w), TSs2Plan(_h, _w), compPurpTS(_h, _w), sobelx(_ssz, _ssz), sobely(_ssz, _ssz), subTSs(_ssz, _ssz), A(_ssz*_ssz, 3), At(3,_ssz*_ssz), AtA(3,3), abc(3), Y(_nn*_nn), ctrl((uint)floor((double)_ssz/2.0), (uint)floor((double)_ssz/2.0)), wMat(_h, _w), sMat(_h*_w, 2), binEvts(10000, 3), alreadyComputedX(_h, _w), alreadyComputedY(_h, _w), polSel(_pol), orientation(_ori), binAcc(_bin), saveOf(_save)
{
    velBuf=_velb;

    TSsData=TSs.data();
    TSs2PlanData=TSs2Plan.data();
    activityData=activity.data();

    alpha=_a;
    threshold=_th;
    tauD=_td;
    tsVal=_tsval;
    neighbor=_nn;
    neighLR=(uint)floor((double)_nn/2.0);
    sobelSz=_ssz;
    sobelLR=(uint)floor((double)_ssz/2.0);
    height=_h;
    width=_w;

    accumulation=_acc;
    vxMean=new double[_h*_w]; memset(vxMean, 0, _h*_w*sizeof(double));
    vyMean=new double[_h*_w]; memset(vyMean, 0, _h*_w*sizeof(double));
    ivxyNData=new uint[_h*_w]; memset(ivxyNData, 0, _h*_w*sizeof(uint));

    //xNeighFlow=new double[_nn*_nn];
    xNeighFlow=new double[_ssz*_ssz];
    //yNeighFlow=new double[_nn*_nn];
    yNeighFlow=new double[_ssz*_ssz];
    ixNeighFlow=0;
    iyNeighFlow=0;

    first=true;

    setSobelFilters(_ssz, sobelx, sobely);
printMatrix(sobelx);
printMatrix(sobely);

    polarity=0;
    activity=0;
    TSs=0;
    TSs2Plan=0;
    compPurpTS=0;
    ctrl=-1;    

    vxMat=_vxMat; *vxMat=0; vxMatData=vxMat->data();
    vyMat=_vyMat; *vyMat=0; vyMatData=vyMat->data();
    mutex=_mutex;
    wMat=0.99;
 
    sMat=0;
    iSMat=0;

    //borneInfX=neighLR+1+sobelLR;
    borneInfX=2*sobelLR+1;
    //borneInfY=neighLR+1+sobelLR;
    borneInfY=2*sobelLR+1;
    borneSupX=height-borneInfX;
    borneSupY=width-borneInfY;

    trans2neigh=new int[2*(4*_ssz-4)];
    iT2N=0;
    for(int i=0; i<_ssz; ++i)
        for(int ii=0; ii<_ssz; ++ii)
        {
            if(i==0 || i==(_ssz-1) || ii==0 || ii==(_ssz-1))
            {
                *(trans2neigh+iT2N*2)=i-(int)sobelLR;
                *(trans2neigh+iT2N*2+1)=ii-(int)sobelLR;
                ++iT2N;
            }
        }
    for(int i=0; i<iT2N; ++i)
        std::cout << *(trans2neigh+i*2) << " " << *(trans2neigh+i*2+1) << endl;

    if(!_src.compare("icub"))
        unmasker=new eventUnmaskICUB();
    else if(!_src.compare("dvs"))
        unmasker=new eventUnmaskDVS128(_type);
    else
        std::cout << "[tsOptFlowThread] Error: Instanciation unmask failed!" << std::endl;
    if(saveOf)
    {
        if(!polSel)
            saveFile.open("eventsFull.txt", ios::out);
        else if(polSel==1)
            saveFile.open("eventsPos.txt", ios::out);
        else if(polSel==-1)
            saveFile.open("eventsNeg.txt", ios::out);
    }
}

tsOptFlowThread::~tsOptFlowThread()
{
    delete[] xNeighFlow;
    delete[] yNeighFlow;
    delete[] activityData;
    delete[] TSsData;
    delete[] TSs2PlanData;
    delete[] vxMatData;
    delete[] vyMatData;
    delete[] vxMean;
    delete[] vyMean;
    delete[] trans2neigh;
    delete unmasker;
}

void tsOptFlowThread::run()
{
    uint refbin;
    uint prefbin;
    while(1)
    {
        //std::cout << "[tsOptFlowThread] Get the data form the buffer" << std::endl;
//        iBinEvts=0;
        int res=0;
        //std::cout << "[tsOptFlowThread] Initialisation..." << endl;
        while(!res)
        {
            res=unmasker->getUmaskedData(addrx, addry, polarity, eye, timestamp);
                //std::cout << "[tsOptFlowThread] res: " << res << std::endl;
            if(polSel && polarity!=polSel) res=0;
        }
        //std::cout << "[tsOptFlowThread] Initialisation done" << std::endl;
        
        if(first)
        {
            first=false;    
            refts=timestamp;
            TSs=timestamp;
            iBinEvts=0;
            refbin=timestamp;
        }
        else
        {
            uint ii=0;
            refbin=((1-alpha)*timestamp+alpha*prefbin);
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
        //if(orientation)
        //    flip();
        binEvts(iBinEvts, 0)=addrx;
        binEvts(iBinEvts, 1)=addry;
        binEvts(iBinEvts, 2)=timestamp;
        iBinEvts++;
            
        //std::cout << "[tsOptFlowThread] Bin filling..." << endl;
        //std::cout << "[tsOptFlowThread] Get the data form the buffer" << std::endl;
        res=0;

        while(refbin+binAcc>=timestamp && iBinEvts<10000)
        {
            res=unmasker->getUmaskedData(addrx, addry, polarity, eye, timestamp);
            if(polSel && polarity!=polSel) res=0;
            if(res)
            {
                //if(orientation)
                //    flip();
                binEvts(iBinEvts, 0)=addrx;
                binEvts(iBinEvts, 1)=addry;
                binEvts(iBinEvts, 2)=timestamp;
                iBinEvts++;
            }
        }
        //std::cout << "[tsOptFlowThread] Bin filled" << endl;
        timestamp=prefbin=refbin;

        //std::cout << "[tsOptFlowThread] evts in the bin: " << iBinEvts << std::endl;
        if(iBinEvts)
        {
            //std::cout << "addrx: " << addrx << ", addry: " << addry << ", polarity: " << polarity << ", eye: " << eye << ", ts: " << timestamp << std::endl;
/*
#ifdef _DEBUG
            std::cout << "[tsOptFlowThread] Update the map of timestamp" << std::endl;
#endif
            updateTimestampMap();
#ifdef _DEBUG
            std::cout << "[tsOptFlowThread] Update the map of activity" << std::endl;
#endif
            updateActivity();
#ifdef _DEBUG
            std::cout << "[tsOptFlowThread] Compute the flow" << std::endl;
#endif
*/
            updateAll();
            //std::cout << "[tsOptFlowThread] Compute the flow" << std::endl;
            compute();
        }
    }
}

void tsOptFlowThread::setSobelFilters(uint _sz, Matrix& _sfx, Matrix& _sfy)
{
    yarp::sig::Vector Sx(_sz);
    yarp::sig::Vector Dx(_sz);
    for(int i=1; i<=_sz; i++)
    {   
        //std::cout << "\t" << i << std::endl;
        //std::cout << "\t" << _sz-1 << "! = " << factorial(_sz-1) << std::endl;
        Sx(i-1)=factorial((_sz-1))/((factorial((_sz-1)-(i-1)))*(factorial(i-1)));
        Dx(i-1)=Pasc(i-1,_sz-2)-Pasc(i-2,_sz-2);
    }
    //Sy=Sx';
    //Dy=Dx';
    _sfx=yarp::math::outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
    _sfy=_sfx.transposed();
}

int tsOptFlowThread::factorial(int _v)
{
    if(_v<=1)
        return 1;
    return _v*factorial(_v-1);
}

int tsOptFlowThread::Pasc(int k, int n)
{
    int P;
    if ( (k>=0) && (k<=n) )
        P=factorial(n)/(factorial(n-k)*factorial(k));
    else
        P=0;
    return P;
}

void tsOptFlowThread::updateAll()
{
    iSMat=0; 
    for(uint i=0; i<height; ++i)
        for(uint ii=0; ii<width; ++ii)
        {
            if(*(TSsData+i*width+ii)+tsVal<timestamp)
            {
                *(TSs2PlanData+i*width+ii)=-1;
//                *(TSsData+i*width+ii)=-1;
                *(activityData+i*width+ii)=0;
            }
/*            if(*(activityData+i*width+ii)>0)
            {
                //std::cout << "[tsOptFlowThread] In the exp(): " << -((double)timestamp-*(TSsData+i*width+ii))/(double)tauD << std::endl;
                //std::cout << "[tsOptFlowThread] With the exp(): " << std::exp(-((double)timestamp-*(TSsData+i*width+ii))/(double)tauD) << std::endl;
                //std::cout << "[tsOptFlowThread] With the a*exp(): " << alpha*std::exp(-((double)timestamp-*(TSsData+i*width+ii))/(double)tauD) << std::endl;

                
//                *(activityData+i*width+ii)=*(activityData+i*width+ii)*alpha*std::exp(-((double)timestamp-*(TSsData+i*width+ii))/tauD);
                if(*(activityData+i*width+ii)>=threshold)
                {
                    sMat(iSMat, 0)=i;
                    sMat(iSMat, 1)=ii;
                    iSMat++;
                    //std::cout << "Threshold reached, flow to compute: " << iSMat << std::endl;
                    //std::cout << "Current activity: " << *(activityData+i*width+ii) << std::endl;
                }
            }*/
        }
    for(uint i=0; i<iBinEvts; i++)
    {
       addrx=binEvts(i, 0); 
       addry=binEvts(i, 1); 
        *(TSsData+addrx*width+addry)=(double)timestamp;
        //*(TSsData+addrx*width+addry)=binEvts(i, 2);
        *(TSs2PlanData+addrx*width+addry)=(double)timestamp;
        //*(TSs2PlanData+addrx*width+addry)=binEvts(i, 2);
        *(activityData+addrx*width+addry)+=1;
        if(*(activityData+addrx*width+addry)>=threshold)
        {
            sMat(iSMat, 0)=addrx;
            sMat(iSMat, 1)=addry;
            iSMat++;
            //std::cout << "Threshold reached, flow to compute: " << iSMat << std::endl;
            //std::cout << "Current activity: " << *(activityData+addrx*width+addry) << std::endl;
        }
    }
    //std::cout << "Number of addr were the activity is suffisant: " << iSMat << std::endl;
//    printMatrix(activity);
}

void tsOptFlowThread::setBuffer(char* _buf, uint _sz)
{
#ifdef _DEBUG
    std::cout << "[tsOptFlowThread] Forward buffer to umask instance" << std::endl;
#endif
    unmasker->setBuffer(_buf, _sz);
}

void tsOptFlowThread::compute()
{
    alreadyComputedX=0;
    alreadyComputedY=0;
    for(uint ixy=0; ixy<iSMat; ++ixy)
    {
        uint x=sMat(ixy,0);
        uint y=sMat(ixy,1);
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
                        //printMatrix(subTSs);  
                        std::cout << "[tsOptFlowThread] Start create plan at (" << xn << ", " << yn << ")" << std::endl;
#endif
                        //uint resp=createPlan(subTSs);
                        dx=dy=0;
                        //uint resp=createPlanAndCompute(subTSs, dx, dy);
                        uint resp=createPlanAndCompute(subTSs, dx, dy, sobelLR, sobelLR, timestamp);
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
                            alreadyComputedX(xn, yn)=dx;
                            alreadyComputedY(xn, yn)=dy;
                            if(dx || dy)
                            {
                                *(xNeighFlow+ixNeighFlow++)=dx;
                                *(yNeighFlow+iyNeighFlow++)=dy;
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
/*
                gsl_sort(xNeighFlow, 1, ixNeighFlow);
                gsl_sort(yNeighFlow, 1, iyNeighFlow);
                double outX=gsl_stats_median_from_sorted_data(xNeighFlow, 1, ixNeighFlow);
                double outY=gsl_stats_median_from_sorted_data(yNeighFlow, 1, iyNeighFlow);
                outX=((outX)*1E-9);
                outY=((outY)*1E-9);
                velBuf.addData((short)x, (short)y, outY, outX, timestamp);
#ifdef _DEBUG
                //std::cout << "addrx: " << x << ", addry: " << y << ", vx: " << *(vxMatData+x*height+y) << ", vy: " << *(vyMatData+x*height+y) << ", ts: " << timestamp << std::endl;
                std::cout << "addrx: " << x << ", addry: " << y << ", vx: " << medianX << ", vy: " << medianY << ", ts: " << timestamp << std::endl;
#endif
*/
                double outX=gsl_stats_mean(xNeighFlow, 1, ixNeighFlow);
                double outY=gsl_stats_mean(yNeighFlow, 1, iyNeighFlow);
                outX=((outX)*1E-9);
                outY=((outY)*1E-9);
                //velBuf.addData((short)x, (short)y, outY, outX, timestamp);
                //std::cout << "Mutex free?" << std::endl;
                *(ivxyNData+x*width+y)+=1;
                *(vxMean+x*width+y)=*(vxMean+x*width+y)+(1/ *(ivxyNData+x*width+y))*(outX-*(vxMean+x*width+y));
                *(vyMean+x*width+y)=*(vyMean+x*width+y)+(1/ *(ivxyNData+x*width+y))*(outY-*(vyMean+x*width+y));

                mutex->wait();
                {
                    //std::cout << "\tMutex took" << std::endl;
                    //velBuf->addData((short)x, (short)y, outY, outX, *(TSsData+x*width+y));
                    velBuf->addData((short)x, (short)y, *(vyMean+x*width+y), *(vxMean+x*width+y), *(TSsData+x*width+y));
                }mutex->post();
                //std::cout << "Mutex freed" << std::endl;
                
                if(saveOf)
                {
                    line2save.str("");
                    //line2save << (short)x << " " << (short)y << " " << outY << " " << outX << " " << timestamp << endl;
                    line2save << (short)x << " " << (short)y << " " << outY << " " << outX << " " << *(TSsData+x*width+y) << endl;
                    saveFile.write( line2save.str().c_str(), line2save.str().size() );
                }
            }
            else
            {
                *(vxMatData+x*height+y)=0;
                *(vyMatData+x*height+y)=0;
            }
        }
    }
}

uint tsOptFlowThread::createPlan(Matrix& _m)
{
#ifdef _DEBUG
    std::cout << "[tsOptFlowThread] Create the plan" << std::endl;
#endif
/*
    std::cout << "[tsOptFlowThread] Full matrix: " << std::endl;
    printMatrix(_m);
    Matrix tmp=_m.submatrix(0, neighLR-1, 0, neighLR-1);
    std::cout << "[tsOptFlowThread] submatrix(0, neighLR-1, 0, neighLR-1)" << std::endl;
    printMatrix(tmp);
    std::cout << "[tsOptFlowThread] is to ctrl: " << (tmp==ctrl) << std::endl;
    tmp=_m.submatrix(0, neighLR-1, neighLR+1, neighbor-1);
    std::cout << "[tsOptFlowThread] submatrix(0, neighLR-1, neighLR+1, neighbor)==ctrl)" << std::endl;
    printMatrix(tmp);
    std::cout << "[tsOptFlowThread] is to ctrl: " << (tmp==ctrl) << std::endl;
    tmp=_m.submatrix(neighLR+1, neighbor-1, 0, neighLR-1);
    std::cout << "[tsOptFlowThread] submatrix(neighLR+1, neighbor, 0, neighLR-1)" << std::endl;
    printMatrix(tmp);
    std::cout << "[tsOptFlowThread] is to ctrl: " << (tmp==ctrl) << std::endl;
    tmp=_m.submatrix(neighLR+1, neighbor-1, neighLR+1, neighbor-1);
    std::cout << "[tsOptFlowThread] submatrix(neighLR+1, neighbor, neighLR+1, neighbor)" << std::endl;
    printMatrix(tmp);
    std::cout << "[tsOptFlowThread] is to ctrl: " << (tmp==ctrl) << std::endl;
*/

    if( !(_m.submatrix(0, neighLR-1, 0, neighLR-1)==ctrl) &&
        !(_m.submatrix(0, neighLR-1, neighLR+1, neighbor-1)==ctrl) &&
        !(_m.submatrix(neighLR+1, neighbor-1, 0, neighLR-1)==ctrl) &&
        !(_m.submatrix(neighLR+1, neighbor-1, neighLR+1, neighbor-1)==ctrl))
    {
        int index=-1;
        uint i;
        uint ii;
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Cover the submatrix" << std::endl;
#endif
        A.resize(neighbor*neighbor, 3);
        Y.resize(neighbor*neighbor);
        for(i=0; i<neighbor; i++)
            for(ii=0; ii<neighbor; ii++)
            {
                if(_m(i, ii)>-1)
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
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Compute the LS-plan" << std::endl;
#endif
        A=A.submatrix(0, index, 0, 2);//A.resize(index, 3);
        Y=Y.subVector(0, index); //Y.resize(index);
        At=A.transposed();
        AtA=At*A;
        abc=pinv(AtA)*At*Y;
#ifdef _DEBUG
        std::cout << "[tsOptFlowThread] Fill the plan with the new values" << std::endl;
#endif
        for(i=0; i<neighbor; i++)
            for(ii=0; ii<neighbor;ii++)
                _m(i, ii)=abc(0)*i+abc(1)*ii+abc(2);
        return 1;
    }
    else
        return 0;
}

uint tsOptFlowThread::createPlanAndCompute(Matrix &_m, double &_dx, double &_dy)
{
#ifdef _DEBUG
    std::cout << "[tsOptFlowThread] Create the plan" << std::endl;
#endif
    uint test=(_m.submatrix(0, sobelLR-1, 0, sobelLR-1)==ctrl)?0:1;
    test+=(_m.submatrix(0, sobelLR-1, sobelLR+1, sobelSz-1)==ctrl)?0:1;
    test+=(_m.submatrix(sobelLR+1, sobelSz-1, 0, sobelLR-1)==ctrl)?0:1;
    test+=(_m.submatrix(sobelLR+1, sobelSz-1, sobelLR+1, sobelSz-1)==ctrl)?0:1;

//    if( !(_m.submatrix(0, sobelLR-1, 0, sobelLR-1)==ctrl) &&
//        !(_m.submatrix(0, sobelLR-1, sobelLR+1, sobelSz-1)==ctrl) &&
//        !(_m.submatrix(sobelLR+1, sobelSz-1, 0, sobelLR-1)==ctrl) &&
//        !(_m.submatrix(sobelLR+1, sobelSz-1, sobelLR+1, sobelSz-1)==ctrl))
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
                if(_m(i, ii)>-1)
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
        abc=pinv(AtA)*At*Y;
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

uint tsOptFlowThread::createPlanAndCompute(Matrix &_m, double &_dx, double &_dy, uint &_curx, uint &_cury, uint &_curts)
{
#ifdef _DEBUG
    std::cout << "[tsOptFlowThread] Create the plan" << std::endl;
#endif
    uint test=(_m.submatrix(0, sobelLR-1, 0, sobelLR-1)==ctrl)?0:1;
    test+=(_m.submatrix(0, sobelLR-1, sobelLR+1, sobelSz-1)==ctrl)?0:1;
    test+=(_m.submatrix(sobelLR+1, sobelSz-1, 0, sobelLR-1)==ctrl)?0:1;
    test+=(_m.submatrix(sobelLR+1, sobelSz-1, sobelLR+1, sobelSz-1)==ctrl)?0:1;

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
                if(_m(i, ii)>-1 && ((i==_curx && ii==_cury) || ((i!=_curx || ii!=_cury) && _m(i, ii)<=(_curts-binAcc))))
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
        abc=pinv(AtA)*At*Y;
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

void tsOptFlowThread::printMatrix(Matrix& _mat)
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

void tsOptFlowThread::flip()
{
    switch(orientation)
    {
        case 90:    addrxBack=addrx;
                    addrx=addry;
                    //addry=height-(addrxBack+1);
                    addry=(addrxBack);
                    break;
        case 180:   addrx=(addrx);
                    //addrx=height-(addrx+1);
                    addry=width-(addry+1);
                    break;
        case 270:   addrxBack=addrx;
                    addrx=width-(addry+1);
                    addry=addrxBack;
                    break;
        default: break;
    }
}
