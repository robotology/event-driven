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
\defgroup iKartBattery iKartBattery
 
@ingroup icub_module  
 
iKart battery manager.
 
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
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/SerialInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <time.h>

#include "graphics.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

struct struct_battery_data
{
	int count;
	int raw_voltage;
	
	double voltage;
	char* timestamp;
};

class CtrlModule: public RFModule
{
public:
	Network		  yarp;
	Gtk::Main	  *gtk_main;
	GraphicsManager* graphics;
    //Port        rpcPort;
	BufferedPort<yarp::os::Bottle> monitor_input;
	sigc::connection m_timer;

public:
    CtrlModule() 
	{
		gtk_main = new Gtk::Main(0,0);
		graphics = new GraphicsManager();
	}

    virtual bool on_timeout()
	{
		static double voltage =0;
		static double current =0;
		static double charge  =0;
		Bottle *b = monitor_input.read(false);
		if (b)
		{
			voltage = b->get(0).asDouble();
			current = b->get(0).asDouble();
			charge  = b->get(0).asDouble();
		}
		else
		{
			fprintf(stderr,"TIMEOUT: unable to receive data from iKart battery manager \n");
		}
		graphics->update_graphics(voltage,current,charge);
		return true;
	}

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

		//check if the yarp networ is running
		if (yarp.checkNetwork()==false)
		{
			return false;
		}

        //rpcPort.open((localName+"/rpc").c_str());
		//attach(rpcPort);

		monitor_input.open("/batteryMonitor:i");
		bool connection_ok = yarp.connect("/ikart/battery:o","/batteryMonitor:i");
		if (!connection_ok)
		{
			fprintf(stderr,"ERROR: unable to connect to iKart battery manager! (looking for port /ikart/battery:o) \n");
			//return false;
		}

  	    m_timer = Glib::signal_timeout().connect(sigc::mem_fun(*this, &CtrlModule::on_timeout), 500);

		//start GTK loop
		gtk_main->run(*graphics);

        return true;
    }

    virtual bool close()
    {
		m_timer.disconnect();
		if (graphics) delete graphics;
		return true;
    }

    virtual double getPeriod()    { return 10.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\tNo options at the moment"<< endl;
        return 0;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}



