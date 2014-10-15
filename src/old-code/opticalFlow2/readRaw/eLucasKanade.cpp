#include <cmath>
#include "eLucasKanade.h"

eLucasKanade::eLucasKanade(int xdim,int ydim,int ndim,double sigma)
{
    mLambda=1.5;

	mXdim=xdim;
	mYdim=ydim;
	mNdim=ndim;
    int side=2*mNdim+1;

    mW.resize(side,side);

    static const double SOBEL_GAIN=0.125*0.125;
    double oneByTwoSigmaSquared=1.0/(2.0*sigma*sigma);

    for (int i=0; i<side; ++i)
    {
        int Di=i-mNdim;
        for (int j=0; j<side; ++j)
        {
            int Dj=j-mNdim;
            mW(i,j)=SOBEL_GAIN*std::exp(-oneByTwoSigmaSquared*double(Di*Di+Dj*Dj));
        }
    }
}

eLucasKanade::~eLucasKanade()
{
}

void eLucasKanade::compute(short *X,short *Y,int N,yarp::sig::Matrix &img1,yarp::sig::Matrix &img0)
{
    int x,y;
    int dx,dy;
    int xa,xb,ya,yb;
    
    double Ix,Iy,It;
    double SwIx2,SwIy2,SwIxIy,SwIxIt,SwIyIt;

	for(int p=0; p<N; ++p)
	{
        x=X[p];
        y=Y[p];
        xa=x-mNdim;
        xb=x+mNdim;
        ya=y-mNdim;
        yb=y+mNdim;

        SwIx2=SwIy2=SwIxIy=SwIxIt=SwIyIt=0.0;

        for (dx=xa; dx<=xb; ++dx)
        {
            for (dy=ya; dy<=yb; ++dy)
            {
                Ix=0.25*(img1(dx+1,dy)-img1(dx-1,dy))+0.125*(img1(dx+1,dy-1)-img1(dx-1,dy-1)+img1(dx+1,dy+1)-img1(dx-1,dy+1));

                Iy=0.25*(img1(dx,dy+1)-img1(dx,dy-1))+0.125*(img1(dx-1,dy+1)-img1(dx-1,dy-1)+img1(dx+1,dy+1)-img1(dx+1,dy-1));
                
                SwIx2 +=Ix*Ix;
                SwIy2 +=Iy*Iy;
                SwIxIy+=Ix*Iy;

                It=img1(dx,dy)-img0(dx,dy);

                SwIxIt-=Ix*It;
                SwIyIt-=Iy*It;
            }
        }

        double s=sqrt((SwIx2-SwIy2)*(SwIx2-SwIy2)+4.0*SwIxIy*SwIxIy);
        double lMin=0.5*(SwIx2+SwIy2-s); // eigenvalue
        
        /*
        double k=SwIx2*SwIy2-SwIxIy*SwIxIy;
        if (k!=0.0)
        {
            k=1.0/k;
            send(x,y,k*(SwIy2*SwIxIt-SwIxIy*SwIyIt),k*(SwIx2*SwIyIt-SwIxIy*SwIxIt));
            continue;
        }
        */

        if (lMin>=mLambda)
        {
            double k=SwIx2*SwIy2-SwIxIy*SwIxIy;
            
            if (k!=0.0)
            {
                k=1.0/k;
                send(x,y,k*(SwIy2*SwIxIt-SwIxIy*SwIyIt),k*(SwIx2*SwIyIt-SwIxIy*SwIxIt));
                continue;
            }
        }
        else
        {
            double lMax=0.5*(SwIx2+SwIy2+s); // eigenvalue
            
            if (lMax>=mLambda)
            {
                double k=SwIx2*SwIy2-SwIxIy*SwIxIy;
        
                if (k!=0.0)
                {
                    k=1.0/k;

                    double vx=k*(SwIy2*SwIxIt-SwIxIy*SwIyIt);
                    double vy=k*(SwIx2*SwIyIt-SwIxIy*SwIxIt);

                    //eigenvector
                    double ux,uy;

		            if (SwIxIy!=0.0)
                    {
                        uy=(lMax-SwIx2)/SwIxIy;
                        ux=1.0/sqrt(1.0+uy*uy);
                        uy*=ux;
                    }
                    else
                    {
                        if (SwIx2>SwIy2)
                        {
				            ux=1.0;
                            uy=0.0;
                        }
			            else
                        {
				            ux=0.0;
                            uy=1.0;
                        }
                    }

                    double uv=vx*ux+vy*uy;
                    send(x,y,uv*ux,uv*uy);
                    continue;
                }
            }
        }

        send(x,y,0.0,0.0);
	}
}

void eLucasKanade::send(int X, int Y, double VX, double VY)
{
    vecBuffer& tmp=mPort.prepare();
    tmp=vecBuffer(X,Y,VX,VY);
    mPort.write();
}
