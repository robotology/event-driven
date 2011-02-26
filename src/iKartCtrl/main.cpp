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

/** 
\defgroup iKartCtrl iKartCtrl
 
@ingroup icub_module  
 
IKart controller (wheels decoupling prototype).
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
@@@TODO
 
\section portsa_sec Ports Accessed
 
@@@TODO
 
\section portsc_sec Ports Created 
 
@@@TODO

\section in_files_sec Input Data Files

@@@TODO

\section out_data_sec Output Data Files 

@@@TODO
 
\section conf_file_sec Configuration Files

@@@TODO

\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "motorsThread.h"
#include "laserThread.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    CtrlThread  *control_thr;
	LaserThread *laser_thr;
    Port        rpcPort;

public:
    CtrlModule() 
	{
		control_thr=0;
		laser_thr=0;
	}

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("ctrlName",Value("ikart")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();

        remoteName=slash+robotName+"/wheels";
        localName=slash+ctrlName;//+"/local/";

		//reads the configuration file
		Property iKartCtrl_options;
		ConstString configFile=rf.findFile("from");		
		if (configFile=="") //--from iKartCtrl.ini
		{
			printf("\nError! Cannot find .ini configuration file. \nBy default I'm searching for iKartCtrl.ini\n");
			return false;
		}
		else
		{
			iKartCtrl_options.fromConfigFile(configFile.c_str());
		}

		// the motor control thread
		bool motors_enabled=true;
		if (rf.check("no_motors"))
		{
			printf("\n'no_motors' option found. Skipping motor control part.\n");
			motors_enabled=false;
		}

		if (motors_enabled==true)
		{
			control_thr=new CtrlThread(20,rf,iKartCtrl_options,remoteName,localName);
			if (!control_thr->start())
			{
				delete control_thr;
				return false;
			}
		}

		// the laser thread
		bool laser_enabled=true;
		if (iKartCtrl_options.check("laser")==false)
		{
			printf("\nLaser configuration not specified. Turning off laser.\n");
			laser_enabled=false;
		}
		if (rf.check("no_laser"))
		{
			printf("\nLaser disabled.\n");
			laser_enabled=false;
		}

		if (laser_enabled==true)
		{
			laser_thr=new LaserThread(20,rf,iKartCtrl_options,remoteName,localName);
			if (!laser_thr->start())
			{
				delete laser_thr;
				return false;
			}
		}

        rpcPort.open((localName+"/rpc").c_str());
		attach(rpcPort);

        return true;
    }

	bool respond(const Bottle& command, Bottle& reply) 
	{
		fprintf(stdout,"rpc respond\n");
		Bottle cmd;
		reply.clear(); 
		
		return true;
	}

    virtual bool close()
    {
		if (control_thr)
		{
			control_thr->stop();
			delete control_thr;
		}
		if (laser_thr)
		{
			laser_thr->stop();
			delete laser_thr;
		}

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);
	rf.setDefaultContext("iKart/conf");
	rf.setDefaultConfigFile("iKartCtrl.ini");

    if (rf.check("help"))
    {
		printf("\n");
        printf("Possible options: \n");
		printf("'no_filter' disables command filtering.\n");
		printf("'no_motors' motor interface will not be opened.\n");
		printf("'no_laser' laser interface will not be opened.\n");
		printf("'no_start' do not automatically enables pwm.\n");
		printf("'laser <filename>' starts the laser with the specified configuration file.\n");
		printf("\n");
		return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
	{
		fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
	}

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;

    return mod.runModule(rf);
}



