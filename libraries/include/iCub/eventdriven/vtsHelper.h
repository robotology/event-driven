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

#ifndef __VTSHELPER__
#define __VTSHELPER__

#include <yarp/os/all.h>
#include <fstream>

namespace ev {

/// \brief helper class to deal with timestamp conversion and wrapping
class vtsHelper {

private:

    int last_stamp;
    unsigned int n_wraps;

public:

    /// the maximum value of the timestamp before a wrap occurs
    static unsigned int max_stamp;
    /// a multiplier to convert an event timestamp to seconds
    static double tsscaler;
    /// a multiplier to convert seconds to an event timestamp
    static double vtsscaler;

    /// \brief constructor
    vtsHelper(): last_stamp(0), n_wraps(0) {}

    /// \brief unwrap a timestamp, given previously unwrapped timestamps
    unsigned long int operator() (int timestamp) {
        if(last_stamp > timestamp)
            n_wraps++;
        last_stamp = timestamp;
        return currentTime();
    }

    /// \brief DEPRECATED - access to max_stamp member variable is public
    static long int maxStamp() { return max_stamp; }
    /// \brief DEPRECATED - access to timestamp conversion member variables is
    /// public
    static double tstosecs() { return tsscaler; }
    /// \brief ask for the current unwrapped time, without updating the time.
    unsigned long int currentTime() { return last_stamp + ((unsigned long int)max_stamp*n_wraps); }

};

class benchmark {
private:

    bool initialised;
    std::ifstream totals_file;
    std::ifstream process_file;

    unsigned long int prevTotal;
    unsigned long int prevProcess;
    double perc;

public:

    benchmark();
    ~benchmark();
    bool isReady();
    double getProcessorUsage();

};

/// \brief an efficient structure for storing sensor resolution
struct resolution {
    unsigned int width:10;
    unsigned int height:10;
};

}

#endif //__VTSHELPER
