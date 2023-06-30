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

#ifndef __VTSHELPER__
#define __VTSHELPER__

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <fstream>
#include <math.h>
#include <vector>
#include "codec.h"

namespace ev {

/// \brief camera values for stereo set-up
enum { CAMERA_LEFT = 0, CAMERA_RIGHT = 1 } ;

#define IS_SKIN(x)      x&0x01000000
#define IS_SKSAMPLE(x)  x&0x00804000
#define IS_SSA(x)       x&0x00004000
#define IS_SSV(x)       x&0x00800000
#define IS_IMU(x)       x&0x02000000
#define IS_AUDIO(x)     x&0x04000000

/// the maximum value of the timestamp before a wrap occurs
extern unsigned int max_stamp;
/// a multiplier to convert an event timestamp to seconds
extern double tsscaler;
/// a multiplier to convert seconds to an event timestamp
extern double vtsscaler;
/// a flag to read if individual event timestamps are enabled
extern bool ts_status;

static double ticksToSeconds(const unsigned int tick)
{
    (void)ticksToSeconds;
    return tick * tsscaler;
}

static unsigned int secondsToTicks(const double seconds)
{
    (void)secondsToTicks;
    return seconds * vtsscaler;
}

static int deltaTicks(const int current_tick, const int prev_tick)
{
    int dt = current_tick - prev_tick;
    if(dt < 0) dt += max_stamp;
    return dt;
}

static double deltaS(const int current_tick, const int prev_tick)
{
    return deltaTicks(current_tick, prev_tick) * tsscaler;
}

static double deltaMS(const int current_tick, const int prev_tick)
{
    (void)deltaMS;
    return deltaS(current_tick, prev_tick) * 1000.0;
}

/// \brief helper class to deal with timestamp conversion and wrapping
class vtsHelper {

private:

    int last_stamp;
    unsigned int n_wraps;

public:

    /// \brief constructor
    vtsHelper(): last_stamp(0), n_wraps(0) {}

    /// \brief unwrap a timestamp, given previously unwrapped timestamps
    unsigned long int operator() (int timestamp) {
        if(last_stamp > timestamp)
            n_wraps++;
        last_stamp = timestamp;
        return currentTime();
    }

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

class imuHelper {
private:
    std::vector<double> bias;
    std::vector<double> gain;
public:

    const static unsigned int _max_value = 32768;

    enum
    {
        ACC_Y = 0, ACC_X = 1, ACC_Z = 2,
        GYR_Y = 3, GYR_X = 4, GYR_Z = 5,
        TEMP  = 6,
        MAG_Y = 7, MAG_X = 8, MAG_Z = 9
    };

    imuHelper()
    {
        bias = {0.0, 0.0, 0.0};
        gain = {9.80665/16384.0, 9.80665/16384.0, 9.80665/16384.0};
    }

    bool configure(const yarp::os::Bottle* config)
    {
        if(!config) {
            yWarning() << "No IMU config given";
            return false;
        } else if(config->size() != 6) {
            yWarning() << "IMU config (6) incorrect size = " << config->size();
            return false;
        }
        bias[ACC_X] = config->get(0).asFloat64();
        gain[ACC_X] = config->get(1).asFloat64();
        bias[ACC_Y] = config->get(2).asFloat64();
        gain[ACC_Y] = config->get(3).asFloat64();
        bias[ACC_Z] = config->get(4).asFloat64();
        gain[ACC_Z] = config->get(5).asFloat64();

        return true;
    }

    double convertToSI(const int value, const int index) const
    {
//        static constexpr double ax_bias = -344.536150;
//        static constexpr double ay_bias = -207.041644;
//        static constexpr double az_bias = -127.343397;

//        static constexpr double ax_gain = 0.000623;
//        static constexpr double ay_gain = 0.000613;
//        static constexpr double az_gain = 0.000599;


        switch(index) {
        case(ACC_X):
        case(ACC_Z):
            return  (value + bias[index]) * gain[index];
        case(ACC_Y):
            return -(value + bias[index]) * gain[index];
        case(GYR_X):
        case(GYR_Z):
            return value * (250.0 * M_PI / (2.0 * 180.0 * 16384.0));
        case(GYR_Y):
            return -(value * (250.0 * M_PI / (2.0 * 180.0 * 16384.0)));
        case(TEMP):
            return (value / 333.87) + (273.15 + 21.0);
        case(MAG_X):
        case(MAG_Z):
            return value * (/*10e6 **/ 4900.0 / 16384.0);
        case(MAG_Y):
            return -(value * (/*10e6 **/ 4900.0 / 16384.0));
        }

        return -1;
    }

};

class refractory_filter
{
private:

    std::vector<double> times;
    std::vector<bool>   pols;
    double period;
    int width;

public:

    void initialise(int width, int height, int seconds)
    {
        times.resize(width * height, 0.0);
        pols.resize(width * height, 0.0);
        period = seconds;
    }

    bool check(ev::AE &v)
    {
        int pass = true;
        int i = v.y * width + v.x;
        if((v.p == pols[i]) && (v.ts - times[i] < period))
            pass = false;
        pols[i] = v.p;
        times[i] = v.ts;

        return pass;
    }
};


}

#endif //__VTSHELPER
