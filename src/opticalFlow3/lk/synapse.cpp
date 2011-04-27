#include <cmath>
#include "synapse.h"

Synapse::Synapse(int xdim,int ydim,int ndim,double sigma,double threshold,double alpha,double tau,int accTime)
{
    for (int x=0; x<XDIM; ++x)
    {
        for (int y=0; y<YDIM; ++y)
        {
            It[x][y]=0.0;
            //mMapN[x][y]=0;
            mMapTime[x][y]=0;
            mImageH[x][y]=0.0;
            mImageL[x][y]=0.0;
            //mMapVx[x][y]=0.0;
            //mMapVy[x][y]=0.0;
            mMapFired[x][y]=0;
        }
    }

    const double SIGMA=1.0;
    for (int x=-NEIGH; x<=NEIGH; ++x)
    {
        for (int y=-NEIGH; y<=NEIGH; ++y)
        {
            mMapGaussian[x+NEIGH][y+NEIGH]=exp(-double(x*x+y*y)/(2.0*SIGMA*SIGMA));
        }
    }
}

Synapse::~Synapse()
{
}

void Synapse::filter(int x0,int y0,int p,unsigned int t)
{
    const unsigned int timeWindow=10000; // 10 ms
    static const double ONE_BY_TAU=1.0/double(timeWindow);
    static const double THR=2.0;

    static double aVx[25],aVy[25];

    double Ix,Iy;
    double SIx2,SIy2,SIxIy;
    double SIxIt,SIyIt;

    double dT;
    double pG;
    double decay;

    int x,y;

    for (x=x0-NEIGH; x<=x0+NEIGH; ++x) if (x>=0 && x<XDIM)
    {
        for (y=y0-NEIGH; y<=y0+NEIGH; ++y) if (y>=0 && y<YDIM)
        {
            pG=p?mMapGaussian[x-x0+NEIGH][y-y0+NEIGH]:-mMapGaussian[x-x0+NEIGH][y-y0+NEIGH];
            dT=double(t-mMapTime[x][y]);
            mMapTime[x][y]=t;
            It[x][y]=pG/(dT>1.0?dT:1.0);
        
            decay=exp(-ONE_BY_TAU*dT);

            if (p)
            {
                mImageH[x][y]=mImageH[x][y]*decay+pG;
                mImageL[x][y]=mImageL[x][y]*decay;
            }
            else
            {
                mImageH[x][y]=mImageH[x][y]*decay;
                mImageL[x][y]=mImageL[x][y]*decay+pG;
            }
        }
    }
    
    int N=0;
    int firePolarity=0;
    
    if (p)
    {   
        if (mImageH[x0][y0]+mImageL[x0][y0]>THR)
        {
            firePolarity= 1;
            SIxIt=SIyIt=0.0;
            SIx2=SIy2=SIxIy=0.0;
        
            for (x=x0-1; x<=x0+1; ++x) if (x>0 && x<XDIM-1)
            {
                for (y=y0-1; y<=y0+1; ++y) if (y>0 && y<YDIM-1)
                {
                    Ix=0.3125*(mImageH[x+1][y]-mImageH[x-1][y])+0.09375*(mImageH[x+1][y-1]-mImageH[x-1][y-1]+mImageH[x+1][y+1]-mImageH[x-1][y+1]);
                    Iy=0.3125*(mImageH[x][y+1]-mImageH[x][y-1])+0.09375*(mImageH[x-1][y+1]-mImageH[x-1][y-1]+mImageH[x+1][y+1]-mImageH[x+1][y-1]);
                    SIx2 +=Ix*Ix;
                    SIy2 +=Iy*Iy;
                    SIxIy+=Ix*Iy;
                    
                    SIxIt+=aVx[N]=-It[x][y]*Ix;
                    SIyIt+=aVy[N]=-It[x][y]*Iy;
                    ++N;
                }
            }
        }
        else
        {
            mMapFired[x0][y0]=0;
            mImageH[x0][y0]=mImageL[x0][y0]=0.0;
        }
    }
    else
    {   
        if (mImageH[x0][y0]+mImageL[x0][y0]<-THR)
        {
            firePolarity=-1;
            SIxIt=SIyIt=0.0;
            SIx2=SIy2=SIxIy=0.0;
            for (x=x0-1; x<=x0+1; ++x) if (x>0 && x<XDIM-1)
            {
                for (y=y0-1; y<=y0+1; ++y) if (y>0 && y<YDIM-1)
                {
                    Ix=0.3125*(mImageL[x+1][y]-mImageL[x-1][y])+0.09375*(mImageL[x+1][y-1]-mImageL[x-1][y-1]+mImageL[x+1][y+1]-mImageL[x-1][y+1]);
                    Iy=0.3125*(mImageL[x][y+1]-mImageL[x][y-1])+0.09375*(mImageL[x-1][y+1]-mImageL[x-1][y-1]+mImageL[x+1][y+1]-mImageL[x+1][y-1]);
                    SIx2 +=Ix*Ix;
                    SIy2 +=Iy*Iy;
                    SIxIy+=Ix*Iy;
                    SIxIt+=aVx[N]=-It[x][y]*Ix;
                    SIyIt+=aVy[N]=-It[x][y]*Iy;
                    ++N;
                }
            }
        }
        else
        {
            mMapFired[x0][y0]=0;
            mImageH[x0][y0]=mImageL[x0][y0]=0.0;
        }
    }

    if (N)
    {
        double m=0.866*sqrt(aVx[4]*aVx[4]+aVy[4]*aVy[4]);
        bool bCoherent=true;
        for (int n=0; n<N; ++n)
        {
            if ((aVx[n]*aVx[4]+aVy[n]*aVy[4])<=m*sqrt(aVx[n]*aVx[n]+aVy[n]*aVy[n]))
            {
                bCoherent=false;
                break;
            }
        }
        if (bCoherent)
        {
            if (mMapFired[x0][y0]*firePolarity<=0)
            {
               double B=SIx2+SIy2;
               double delta=sqrt(SIx2*SIx2+SIy2*SIy2-2.0*SIx2*SIy2+4.0*SIxIy*SIxIy);
                        
               if ((B-delta)<0.1*(B+delta))
               {
                   mMapFired[x0][y0]=firePolarity;

                   vecBuffer& tmp=mPort.prepare();
                   tmp=vecBuffer(x0,y0,SIxIt,SIyIt);
                   mPort.write();
               }
           }
        }
    }
}
