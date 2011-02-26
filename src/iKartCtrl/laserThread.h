/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
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

#ifndef LASER_THREAD_H
#define LASER_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#define PRINT_STATUS_PER    0.5     // [s]

class LaserThread: public yarp::os::RateThread
{
	protected:
	Property iKartCtrl_options;

	ResourceFinder      &rf;
	PolyDriver          *laser_driver;
	BufferedPort<yarp::sig::Vector> port_laser_output;
	IAnalogSensor       *iLaser;
	string remoteName;
    string localName;

	public:
    
	LaserThread(unsigned int _period, ResourceFinder &_rf, Property options,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
			   iKartCtrl_options (options),
               remoteName(_remoteName), localName(_localName) 
	{
	}

    virtual bool threadInit()
    {
		ConstString laser_filename = iKartCtrl_options.find("laser").asString();
		ConstString laser_config_filename =rf.findFile(laser_filename);		
		if (laser_config_filename=="") 
		{
			printf("\nError! Unable to locate .ini laser configuration file. \nLooking for %s\n",laser_config_filename.c_str());
			return false;
		}
		else
		{
			printf("\nOpening the laser interface...\n");
			Property laser_options;
			laser_options.fromConfigFile(laser_config_filename.c_str());
			laser_options.put("CONFIG_PATH",rf.getContextPath().c_str());

			// open the laser scanner driver
			laser_driver=new PolyDriver;
			laser_options.put("device","laserHokuyo");
			if (!laser_driver->open(laser_options))
			{
				fprintf(stderr,"ERROR: cannot open laser driver...\n");
				delete laser_driver;    
				return false;
			}
			//open the interface for the laser
			bool laser_ok = laser_driver->view(iLaser);
			if(!laser_ok)
			{
				fprintf(stderr,"ERROR: cannot view the laser interface\nreturning...\n");
				return false;
			}
			// open the laser output port
			port_laser_output.open((localName+"/laser:o").c_str());
		}

		return true;
	}

	virtual void run()
	{		
		yarp::sig::Vector laser_data;
		//fprintf(stderr,"before laser reading\n");
		int res = iLaser->read(laser_data);
		//fprintf(stderr,"after laser reading\n");
		if (res == yarp::dev::IAnalogSensor::AS_OK)
		{
			yarp::sig::Vector &plaser_data=port_laser_output.prepare();
			plaser_data=laser_data;
			//lastStateStamp.update();
			//port_laser_data.setEnvelope(lastStateStamp);
			port_laser_output.write();
		}
		else
		{
			fprintf(stderr,"Error reading laser data, code: %d\n", res);
		}
	}

	virtual void threadRelease()
    {    
        delete laser_driver;

		port_laser_output.interrupt();
        port_laser_output.close();
    }
};

#endif