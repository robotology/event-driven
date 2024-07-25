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

#include "event-driven/vis/filters.h"

namespace ev {

void spatialFilter::initialise(int height, int width, double period = 0.1, int range = 1)
{
    this->range = range;
    this->period = period;
    for(auto& sae : saes)
        sae = cv::Mat::zeros(height +2*range, width+2*range, CV_64F);
}

bool spatialFilter::check(const AE& v, const double ts)
{
    static int fr = 2*range+1; 
    bool pass = true;
    if(ts - period > saes[v.p].at<double>(v.y+range, v.x+range))
        pass = false;
    saes[v.p]({(int)v.x, (int)v.y, fr, fr}) = ts;
    return pass;
}


vNoiseFilter::vNoiseFilter() : x_sfilter(false), x_tfilter(false), t_sfilter(0),
    s_sfilter(1), t_tfilter(0) {}

void vNoiseFilter::initialise(unsigned int width, unsigned int height)
{
    res.height = height;
    res.width = width;
    SAE = cv::Mat::zeros(height, width, CV_64F);
    POL = cv::Mat::ones(height, width, CV_8U)*255;
    initialised = true;
}

const bool& vNoiseFilter::active()
{
    return initialised;
}

void vNoiseFilter::use_temporal_filter(double t_param)
{
    x_tfilter = true;
    t_tfilter = t_param;
}

void vNoiseFilter::use_spatial_filter(double t_param, unsigned int s_param)
{
    x_sfilter = true;
    t_sfilter = t_param;
    s_sfilter = s_param;
}

bool vNoiseFilter::check(int x, int y, int p, double t)
{

    auto add = true;

    if(x_tfilter) {
        if(p == POL.at<uint8_t>(y, x)) {
            if(t - SAE.at<double>(y, x) < t_tfilter) {
                SAE.at<double>(y, x) = t;
                return false;
            }
        }
    }

    if(x_sfilter) {
        add = false;
        auto xl = std::max(x-s_sfilter, 0);
        auto xh = std::min(x+s_sfilter+1, (int)(res.width)); //+1 becuase I use < sign
        auto yl = std::max(y-s_sfilter, 0);
        auto yh = std::min(y+s_sfilter+1, (int)(res.height)); //+1 becuase I use < sign

        for(auto xi = xl; xi < xh; ++xi) {
            for(auto yi = yl; yi < yh; ++yi) {
                double dt = t - SAE.at<double>(yi, xi);
                if(dt < t_sfilter) {
                    add = true;
                    break;
                }
            }
        }
    }

    POL.at<uint8_t>(y, x) = p;
    SAE.at<double>(y, x) = t;

    return add;
}

}


