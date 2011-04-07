#include <cmath>
#include "synapse.h"

Synapse::Synapse(int xdim,int ydim,int ndim,double sigma,double threshold,double alpha,double tau,int accTime)
{
    mXdim=xdim;
    mYdim=ydim;
    mNdim=ndim;
    int side=2*mNdim+1;

    mThreshold=threshold;
    mAlpha=alpha;
    mOneByTau=1.0/tau;
    mAccTime=accTime;
    
    mImage0.resize(mXdim,mYdim);
    mImage0.zero();

    mImage1.resize(mXdim,mYdim);		
    mImage1.zero();

	mPotential.resize(mXdim,mYdim);
    mPotential.zero();
   
	mTime.resize(mXdim,mYdim);		   
    mTime.zero();
    
    X=new short[mXdim*mYdim];
    Y=new short[mXdim*mYdim];
   
    mGaussianMask.resize(side,side);
    
    double oneByTwoSigmaSquared=1.0/(2.0*sigma*sigma);
    for (int i=0; i<side; ++i)
    {
        int Di=i-mNdim;
        for (int j=0; j<side; ++j)
        {
            int Dj=j-mNdim;
            mGaussianMask(i,j)=std::exp(-oneByTwoSigmaSquared*double(Di*Di+Dj*Dj));
        }
    }

	mTimeStamp=0;
	
    //////////////////////////////////////////////////
    objFlow = new eLucasKanade(mXdim,mYdim,mNdim,1.0);
    //////////////////////////////////////////////////
}

Synapse::~Synapse()
{
    delete [] X;
    delete [] Y;
}

void Synapse::filter(int x0,int y0,int p,unsigned int t)
{
	if (!mTimeStamp) mTimeStamp=t;

    int xa=x0-mNdim;
    int xb=x0+mNdim;
    int ya=y0-mNdim;
    int yb=y0+mNdim;

    int dx;
    int dy;
    int nbMatch=0;
    double dt=double(t);
    double k=double(5*p);
    for (int x=xa; x<=xb; ++x)
    {
        dx=x-xa;
        for (int y=ya; y<=yb; ++y)
        {
            dy=y-ya;
        
            mImage1(x,y)+=k*mGaussianMask(dx,dy);

            mPotential(x,y)*=mAlpha*exp(-mOneByTau*(dt-mTime(x,y)));
            mPotential(x,y)+=mGaussianMask(dx,dy);

            if (mPotential(x,y)>=mThreshold)
            {
                mPotential(x,y)=-0.5;
                X[nbMatch  ]=x;
                Y[nbMatch++]=y;
            }

            mTime(x,y)=dt;
        }
    }

    if (nbMatch)
    {
        objFlow->compute(X,Y,nbMatch,mImage1,mImage0);
    }

	if (t>=mTimeStamp+mAccTime)
	{
		mImage0=mImage1;
		mTimeStamp=t;
	}
}
