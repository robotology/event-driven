/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
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

#include "iCub/eventdriven/vtsHelper.h"

namespace ev {

#ifdef TIME32BIT
    //long int vtsHelper::max_stamp = 2147483647; //2^31
    long int vtsHelper::max_stamp = 0x01FFFFFF; //2^25
#else
    long int vtsHelper::max_stamp = 16777215; //2^24
#endif

#ifdef TENBITCODEC
    double vtsHelper::tsscaler = 0.000000080;
#else
    double vtsHelper::tsscaler = 0.000000128;
#endif

#ifdef TENBITCODEC
    double vtsHelper::vtsscaler = 12500000;
#else
    double vtsHelper::vtsscaler = 7812500;
#endif

}
