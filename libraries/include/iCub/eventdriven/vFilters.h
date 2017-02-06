#ifndef __VFILTER__
#define __VFILTER__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/Image.h>

namespace ev {

class vNoiseFilter
{
private:

    int Tsize;
    int Ssize;

    yarp::sig::ImageOf <yarp::sig::PixelInt> TSleftL;
    yarp::sig::ImageOf <yarp::sig::PixelInt> TSleftH;
    yarp::sig::ImageOf <yarp::sig::PixelInt> TSrightL;
    yarp::sig::ImageOf <yarp::sig::PixelInt> TSrightH;

public:

    vNoiseFilter() : Tsize(0), Ssize(0) {}

    void initialise(double width, double height, int Tsize, unsigned int Ssize)
    {
        TSleftL.resize(width + 2 * Ssize, height + 2 * Ssize);
        TSleftH.resize(width + 2 * Ssize, height + 2 * Ssize);
        TSrightL.resize(width + 2 * Ssize, height + 2 * Ssize);
        TSrightH.resize(width + 2 * Ssize, height + 2 * Ssize);

        TSleftL.zero();
        TSleftH.zero();
        TSrightL.zero();
        TSrightH.zero();

        this->Tsize = Tsize;
        this->Ssize = Ssize;
    }

    bool check(int x, int y, int p, int c, int ts)
    {
        if(!Ssize) return false;

        yarp::sig::ImageOf<yarp::sig::PixelInt> *active = 0;
        if(c == 0) {
            if(p == 0) {
                active = &TSleftL;
            } else if(p == 1) {
                active = &TSleftH;
            }
        } else if(c == 1) {
            if(p == 0) {
                active = &TSrightL;
            } else if(p == 1) {
                active = &TSrightH;
            }
        }
        if(!active) return false;

        x += Ssize;
        y += Ssize;


        bool add = false;
        (*active)(x, y) = ts;
        for(int xi = x - Ssize; xi <= x + Ssize; xi++) {
            for(int yi = y - Ssize; yi <= y + Ssize; yi++) {
                int dt = ts - (*active)(xi, yi);
                if(dt < 0) {
                    dt += vtsHelper::maxStamp();
                    (*active)(xi, yi) -= vtsHelper::maxStamp();
                }
                if(dt && dt < Tsize) {
                    add = true;
                    break;
                }
            }
        }

        return add;
    }

};


}

#endif
