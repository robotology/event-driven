#ifndef __IKARTNAV_VEC2D_H__
#define __IKARTNAV_VEC2D_H__

#define _USE_MATH_DEFINES
#include <math.h>

class Vec2D  
{
public:
	Vec2D():x(0.0),y(0.0){}
	Vec2D(double x0,double y0):x(x0),y(y0){}
    explicit Vec2D(double dH)
    {
        dH*=DEG2RAD;
        x=cos(dH);
        y=sin(dH);
    }
	~Vec2D(){}

	double mod2()const{ return x*x+y*y; }
	double mod()const{ return sqrt(x*x+y*y); }

    Vec2D rot(double alfa)
    {
        alfa*=Vec2D::DEG2RAD;
        double cs=cos(alfa);
        double sn=sin(alfa);
        return Vec2D(cs*x-sn*y,sn*x+cs*y);
    }

	Vec2D rotLeft()const{ return Vec2D(-y,x); }
	Vec2D rotRight()const{ return Vec2D(y,-x); }

	Vec2D norm(double dl=1.0) const
	{ 
		double dm=mod();
		
		if (dm!=0.0)
			return *this*(dl/dm);
	    else
			return Vec2D(0.0,0.0);
	}

	void normalize(double dl=1.0)
	{ 
		double dm=mod();
		
		if (dm!=0.0)
		{
			x*=dl/=dm;
			y*=dl;
		}
		else
			x=y=0.0;
	}

	double arg() const
	{
        static const double RAD2DEG=1.0/DEG2RAD;
		return RAD2DEG*atan2(y,x);
	}

	inline Vec2D operator+(const Vec2D& p)const{ return Vec2D(x+p.x,y+p.y); }
	inline Vec2D operator-(const Vec2D& p)const{ return Vec2D(x-p.x,y-p.y); }
	inline Vec2D operator-()const{ return Vec2D(-x,-y); }
	inline Vec2D operator*(double a)const{ return Vec2D(x*a,y*a); }
	inline double operator^(const Vec2D& p)const{ return x*p.y-y*p.x; }
	inline Vec2D operator/(double a){ a=1.0/a; return Vec2D(x*a,y*a); }
	inline Vec2D operator+=(const Vec2D &p){ x+=p.x; y+=p.y; return *this; }
	inline Vec2D operator-=(const Vec2D &p){ x-=p.x; y-=p.y; return *this; }
	inline Vec2D operator*=(double a){ x*=a; y*=a; return *this; }
	inline Vec2D operator/=(double a){ a=1.0/a; x*=a; y*=a; return *this; }
	inline double operator*(const Vec2D& p)const{ return x*p.x+y*p.y; }

    inline bool operator==(const Vec2D& p)const{ return x==p.x && y==p.y; }
    inline bool operator!=(const Vec2D& p)const{ return x!=p.x || y!=p.y; }

    double x,y;
    static const Vec2D zero;
    static const double DEG2RAD;
};

inline Vec2D operator*(double a,const Vec2D& p){ return Vec2D(a*p.x,a*p.y); }

#endif
