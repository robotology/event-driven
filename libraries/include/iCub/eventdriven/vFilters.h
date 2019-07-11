/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VFILTER__
#define __VFILTER__

#include <yarp/sig/Image.h>

namespace ev {

/// \brief an efficient event-based salt and pepper filter
class vNoiseFilter
{
private:

    bool x_sfilter;
    bool x_tfilter;

    int t_sfilter;
    int s_sfilter;
    int t_tfilter;

    yarp::sig::ImageOf <yarp::sig::PixelInt> TSleftL;
    yarp::sig::ImageOf <yarp::sig::PixelInt> TSleftH;
    yarp::sig::ImageOf <yarp::sig::PixelInt> TSrightL;
    yarp::sig::ImageOf <yarp::sig::PixelInt> TSrightH;

public:

    /// \brief constructor
    vNoiseFilter() : x_sfilter(false), x_tfilter(false), t_sfilter(0),
        s_sfilter(0), t_tfilter(0) {}

    /// \brief initialise the sensor size and the filter parameters.
    void initialise(double width, double height)
    {

        TSleftL.resize(width + 2 * s_sfilter, height + 2 * s_sfilter);
        TSleftH.resize(width + 2 * s_sfilter, height + 2 * s_sfilter);
        TSrightL.resize(width + 2 * s_sfilter, height + 2 * s_sfilter);
        TSrightH.resize(width + 2 * s_sfilter, height + 2 * s_sfilter);

        TSleftL.zero();
        TSleftH.zero();
        TSrightL.zero();
        TSrightH.zero();
    }

    void use_temporal_filter(int t_param)
    {
        x_tfilter = true;
        t_tfilter = t_param;
    }

    void use_spatial_filter(int t_param, unsigned int s_param)
    {
        x_sfilter = true;
        t_sfilter = t_param;
        s_sfilter = s_param;
    }

    /// \brief classifies the event as noise or signal
    /// \returns false if the event is noise
    bool check(int x, int y, int p, int c, int ts)
    {

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

        x += s_sfilter;
        y += s_sfilter;

        if(x_tfilter) {
            int dt = ts - (*active)(x, y);
            if(dt < 0)
                dt += vtsHelper::max_stamp;
            if(dt < t_tfilter)
                return false;
        }

        (*active)(x, y) = ts;

        auto add = false;

        if(x_sfilter) {
            for(int xi = x - s_sfilter; xi <= x + s_sfilter; xi++) {
                for(int yi = y - s_sfilter; yi <= y + s_sfilter; yi++) {
                    int dt = ts - (*active)(xi, yi);
                    if(dt < 0) {
                        dt += vtsHelper::max_stamp;
                        (*active)(xi, yi) -= vtsHelper::max_stamp;
                    }
                    if(dt && dt < t_sfilter) {
                        add = true;
                        break;
                    }
                }
            }
        }

        return add;
    }

};


}

#endif
