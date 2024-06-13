/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
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

#pragma once

#include <opencv2/opencv.hpp>
#include "event-driven/core.h"

namespace ev {

/// \brief an efficient event-based salt and pepper filter
class vNoiseFilter
{
private:

    bool x_sfilter;
    bool x_tfilter;

    double t_sfilter;
    int s_sfilter;
    double t_tfilter;

    cv::Mat SAE;
    cv::Mat POL;

    resolution res;
    bool initialised{false};

public:

    /// \brief constructor
    vNoiseFilter();

    /// \brief initialise the sensor size and the filter parameters.
    void initialise(unsigned int width, unsigned int height);

    const bool& active();

    /// \brief filter using temporal coincidence
    void use_temporal_filter(double t_param);

    /// \brief filter using spatial coincidence
    void use_spatial_filter(double t_param, unsigned int s_param = 1);

    /// \brief classifies the event as noise or signal
    /// \returns false if the event is noise
    bool check(int x, int y, int p, double t);

};

class spatialFilter
{
private:
    std::array<cv::Mat, 2> saes;
    double period;
    double range;

public:
    spatialFilter() {};
    void initialise(int height, int width, double period, int range);
    bool check(const AE& v, const double ts);

};


}
