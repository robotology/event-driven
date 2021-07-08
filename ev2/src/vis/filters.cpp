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


vNoiseFilter::vNoiseFilter() : x_sfilter(false), x_tfilter(false), t_sfilter(0),
    s_sfilter(1), t_tfilter(0) {}

void vNoiseFilter::initialise(unsigned int width, unsigned int height)
{
    res.height = height;
    res.width = width;
    SAE = cv::Mat::zeros(height, width, CV_32S);
    POL = cv::Mat::zeros(height, width, CV_8U);
}

void vNoiseFilter::use_temporal_filter(int t_param)
{
    x_tfilter = true;
    t_tfilter = t_param;
}

void vNoiseFilter::use_spatial_filter(int t_param, unsigned int s_param)
{
    x_sfilter = true;
    t_sfilter = t_param;
    s_sfilter = s_param;
}

bool vNoiseFilter::check(int x, int y, int p, int ts)
{

    auto add = true;

    if(x_tfilter) {
        if(p == POL.at<uint8_t>(y, x)) {
            int dt = ts - SAE.at<uint32_t>(y, x);
            if(dt < 0)
                dt += ev::max_stamp;
            if(dt < t_tfilter) {
                SAE.at<uint32_t>(y, x) = ts;
                return false;
            }
        }
    }

    POL.at<uint8_t>(y, x) = p;
    SAE.at<uint32_t>(y, x) = ts;

    if(x_sfilter) {
        add = false;
        auto xl = std::max(x-s_sfilter, 0);
        auto xh = std::min(x+s_sfilter+1, (int)(res.width)); //+1 becuase I use < sign
        auto yl = std::max(y-s_sfilter, 0);
        auto yh = std::min(y+s_sfilter+1, (int)(res.height)); //+1 becuase I use < sign

        for(auto xi = xl; xi < xh; ++xi) {
            for(auto yi = yl; yi < yh; ++yi) {
                int dt = ts - SAE.at<uint32_t>(yi, xi);
                if(dt < 0) {
                    dt += ev::max_stamp;
                    SAE.at<uint32_t>(yi, xi) -= ev::max_stamp;
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

}


