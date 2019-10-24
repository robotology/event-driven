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
#include "event-driven/vtsHelper.h"

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

    yarp::sig::ImageOf <yarp::sig::PixelInt> SAE;
    yarp::sig::ImageOf <yarp::sig::PixelMono> POL;

    resolution res;

public:

    /// \brief constructor
    vNoiseFilter() : x_sfilter(false), x_tfilter(false), t_sfilter(0),
        s_sfilter(0), t_tfilter(0) {}

    /// \brief initialise the sensor size and the filter parameters.
    void initialise(unsigned int width, unsigned int height)
    {
        res.height = height;
        res.width = width;

        SAE.resize(width, height);
        POL.resize(width, height);

        SAE.zero();
        POL.zero();
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
    bool check(int x, int y, int p, int ts)
    {

        auto add = true;

        if(x_tfilter) {
            if(p == POL(x, y)) {
                int dt = ts - SAE(x, y);
                if(dt < 0)
                    dt += vtsHelper::max_stamp;
                if(dt < t_tfilter) {
                    SAE(x, y) = ts;
                    return false;
                }
            }
        }

        POL(x, y) = p;
        SAE(x, y) = ts;

        if(x_sfilter) {
            add = false;
            auto xl = std::max(x-s_sfilter, 0);
            auto xh = std::min(x+s_sfilter+1, (int)(res.width)); //+1 becuase I use < sign
            auto yl = std::max(y-s_sfilter, 0);
            auto yh = std::min(y+s_sfilter+1, (int)(res.height)); //+1 becuase I use < sign

            for(auto xi = xl; xi < xh; ++xi) {
                for(auto yi = yl; yi < yh; ++yi) {
                    int dt = ts - SAE(xi, yi);
                    if(dt < 0) {
                        dt += vtsHelper::max_stamp;
                        SAE(xi, yi) -= vtsHelper::max_stamp;
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
