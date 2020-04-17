/*
 *   Copyright (C) 2020 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <yarp/os/all.h>
#include "event-driven/all.h"
#include <vector>
#include <iomanip>

using namespace ev;
using namespace yarp::os;
using std::vector;

class qadIMUcal : public RFModule, public Thread {

private:

    vReadPort< vector<IMUevent>  > imu_port;
    vector<double> imu_state;

    struct inc_mean
    {
        double value = 0.0;
        int n = 0;
    };
    vector<inc_mean> mean_vals;
    enum axis_number {
        INVALID = -1, X_UP = 0, X_DOWN = 1, Y_UP = 2, Y_DOWN = 3, Z_UP = 4, Z_DOWN = 5
    };

    double threshold;
    static constexpr double min_grav = 15000;

public:

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/qadIMUcal")).asString()).c_str());

        //open io ports
        if(!imu_port.open(getName() + "/IMU:i")) {
            yError() << "Could not open input port";
            return false;
        }

        //threshold
        threshold = rf.check("threshold", Value(20)).asDouble();

        //inilialize the vector which saves the values
        imu_state.resize(10, 0.0);
        mean_vals.resize(6);

        //start the asynchronous and synchronous threads
        return Thread::start();
    }

    virtual double getPeriod()
    {
        return 0.1;
    }

    bool interruptModule()
    {
        std::cout << std::endl;
        return Thread::stop();
    }

    void onStop()
    {
        imu_port.close();
    }


    virtual bool updateModule()
    {        
        std::cout << std::endl << "---------------" << std::endl;
        std::cout << "Calibrate the IMU by pointing each axis directly up. The other two axis "
                     "must therefore be close to 0. Once each of the 6 axes have > 100 samples "
                     "the calibrated values can be copied to a .ini file and read asList()" << std::endl;
        std::cout << "RAW ACCELEROMETER" << std::endl;
        std::cout << "AXIS 0  :  1  :  2\t\t| Current values and samples" << std::endl;

        std::cout << std::fixed << std::setprecision(2);
        std::cout << imu_state[imuHelper::ACC_X] << " " <<
                     imu_state[imuHelper::ACC_Y] << " " <<
                     imu_state[imuHelper::ACC_Z] << "      | ";
        std::cout << mean_vals[X_UP].value << " " <<
                     mean_vals[X_UP].n << " ";
        std::cout << mean_vals[X_DOWN].value << " " <<
                     mean_vals[X_DOWN].n << " | ";
        std::cout << mean_vals[Y_UP].value << " " <<
                     mean_vals[Y_UP].n << " ";
        std::cout << mean_vals[Y_DOWN].value << " " <<
                     mean_vals[Y_DOWN].n << " | ";
        std::cout << mean_vals[Z_UP].value << " " <<
                     mean_vals[Z_UP].n << " ";
        std::cout << mean_vals[Z_DOWN].value << " " <<
                     mean_vals[Z_DOWN].n << " | ";
        std::cout << std::endl;

        double xbias = (mean_vals[X_UP].value + mean_vals[X_DOWN].value) * 0.5;
        double ybias = (mean_vals[Y_UP].value + mean_vals[Y_DOWN].value) * 0.5;
        double zbias = (mean_vals[Z_UP].value + mean_vals[Z_DOWN].value) * 0.5;
        double xgain = 9.80665 / (mean_vals[X_UP].value + xbias);
        double ygain = 9.80665 / (mean_vals[Y_UP].value + ybias);
        double zgain = 9.80665 / (mean_vals[Z_UP].value + zbias);



        std::cout << "(X(bias) X(gain) | Y(bias) Y(gain) | Z(bias) Z(gain))" << std::endl;

        std::cout << std::setprecision(9);
        std::cout << "(";
        if(mean_vals[X_UP].n && mean_vals[X_DOWN].n)
            std::cout << std::setprecision(1) << xbias << " " << std::setprecision(9) << xgain << " ";
        else
            std::cout << "  -   -  ";
        if(mean_vals[Y_UP].n && mean_vals[Y_DOWN].n)
            std::cout << ybias << " " << ygain << " ";
        else
            std::cout << "  -   -  ";
        if(mean_vals[Z_UP].n && mean_vals[Z_DOWN].n)
            std::cout << zbias << " " << zgain;
        else
            std::cout << "  -   -  ";
        std::cout << ")" << std::endl;


        return Thread::isRunning();
    }

    void run()
    {
        Stamp yarpstamp_imu;

        while(true) {

            const vector<IMUevent> * q = imu_port.read(yarpstamp_imu);
            if(!q || Thread::isStopping()) return;

            for(const IMUevent &v : *q)
            {
                imu_state[v.sensor] = v.value;
                if(v.sensor != 9) continue;

                axis_number imi = INVALID;
                double value = 0.0;


                if(fabs(imu_state[imuHelper::ACC_X]) < threshold &&
                   fabs(imu_state[imuHelper::ACC_Y]) < threshold)
                {
                    value = imu_state[imuHelper::ACC_Z];
                    if(value > min_grav) imi = Z_UP;
                    else if(value < -min_grav) imi = Z_DOWN;
                }

                else if(fabs(imu_state[imuHelper::ACC_X]) < threshold &&
                        fabs(imu_state[imuHelper::ACC_Z]) < threshold)
                {
                    value = imu_state[imuHelper::ACC_Y];
                    if(value > min_grav) imi = Y_UP;
                    else if(value < -min_grav) imi = Y_DOWN;
                }

                else if(fabs(imu_state[imuHelper::ACC_Y]) < threshold &&
                        fabs(imu_state[imuHelper::ACC_Z]) < threshold)
                {
                    value = imu_state[imuHelper::ACC_X];
                    if(imu_state[imuHelper::ACC_X] > min_grav) imi = X_UP;
                    else if(imu_state[imuHelper::ACC_X] < -min_grav) imi = X_DOWN;
                }

                if(imi != INVALID)
                {
                    inc_mean &m = mean_vals[imi];
                    m.value += (value - m.value) / ++m.n;
                }


            }
        }
    }
};


int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return -1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "qadIMUcal.ini" );
    rf.configure( argc, argv );

    /* create the module */
    qadIMUcal instance;
    return instance.runModule(rf);
}
