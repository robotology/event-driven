/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#ifndef __VFILTER__
#define __VFILTER__

#include <yarp/sig/Image.h>

namespace ev {

/// \brief an efficient event-based salt and pepper filter
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

    /// \brief constructor
    vNoiseFilter() : Tsize(0), Ssize(0) {}

    /// \brief initialise the sensor size and the filter parameters.
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

    /// \brief classifies the event as noise or signal
    /// \returns false if the event is noise
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
                    dt += vtsHelper::max_stamp;
                    (*active)(xi, yi) -= vtsHelper::max_stamp;
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
