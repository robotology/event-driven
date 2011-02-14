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
#include <cstring>


YARP_DECLARE_DEVICES(icubmod)

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

class CtrlThread: public RateThread
{

private:		 
	bool                logEnable;
	bool                verboseEnable;
	bool                screenEnable;
	char                log_buffer[255];
	FILE				*logFile;
	bool                yarp_found;
    Network				yarp;

protected:
    ResourceFinder      &rf;
    PolyDriver          driver;
	ISerialDevice       *pSerial;
	char				serial_buff[255];
	Port                port_battery_output;
	Port                port_shutdown;

    string remoteName;
    string localName;

	struct_battery_data battery_data;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) 
	{
		//yarp.setVerbosity(-1);
		logEnable=false;
		for (int i=0; i<255; i++) serial_buff[i]=0;
    }


    virtual bool threadInit()
    {
		//open the logfile if requested
		logEnable=rf.check("logToFile");
		verboseEnable=rf.check("verbose");
		screenEnable=rf.check("screen");

		//check for alternate COM ports
		ConstString COMport = rf.check("COMport",Value("none"),"Name of the COM port (i.e. COM2, /ttyUSB0 etc.)").asString();

		//serial port configuration parameters
	    Property prop;
		prop.put("device",       "serialport");
		prop.put("verbose",      "0");
		if (COMport=="none")
		{
			//default values
			#ifdef WIN32
				prop.put("comport",      "COM2");
			#else
				prop.put("comport",      "/ttyUSB0");
			#endif
		}
		else
		{
			prop.put("comport",      COMport.c_str());
		}
		prop.put("baudrate",     115200);
		prop.put("xonlim ",      0);
		prop.put("xofflim",      0);
		prop.put("readmincharacters", 1);
		prop.put("readtimeoutmsec",   2);
		prop.put("paritymode",   "none");
		prop.put("ctsenb",       0);
		prop.put("rtsenb",       0);
		prop.put("xinenb",       0);
		prop.put("xoutenb",      0);
		prop.put("modem",        0);
		prop.put("rcvenb",       0);
		prop.put("dsrenb",       0);
		prop.put("dtrdisable",   0);
		prop.put("databits",     8);
		prop.put("stopbits",     1);

		if (logEnable)
		{
			fprintf(stderr, "writing to log file batteryLog.txt\n");
			logFile = fopen("batteryLog.txt","w");
		}

		//open serial port driver
		driver.open(prop);
		if (!driver.isValid())
		{
			fprintf(stderr, "Error opening PolyDriver check parameters\n");
			return false;
		}
		driver.view(pSerial);
		    
		if (!pSerial)
		{
			fprintf(stderr, "Error opening serial driver. Device not available\n");
			return false;
		}
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stderr, "Thread started successfully\n");
        else
            fprintf(stderr, "Thread did not start\n");
    }

	void notify_message(string msg)
	{
	#ifdef WIN32
		fprintf(stderr,msg.c_str());
	#else
		fprintf(stderr,msg.c_str());
		string cmd = "wall " +msg;
		system(cmd.c_str());
	#endif
	}

	void emergency_shutdown(string msg)
	{
	#ifdef WIN32
		string cmd = "shutdown /s /t 120 /c "+msg;
		fprintf(stderr,msg.c_str());
		system(cmd.c_str());
	#else
		string cmd = "shutdown -h -t 120 "+msg;
		fprintf(stderr,msg.c_str());
		system(cmd.c_str());
	#endif
	}

	void stop_robot(string quit_port)
	{
		//typical quit_port:
		// "/icub/quit"
		// "/ikart/quit"
		if (yarp_found)
		{
			port_shutdown.open((localName+"/shutdown").c_str());
			yarp.connect((localName+"/shutdown").c_str(),quit_port.c_str());
			Bottle bot;
			bot.addString("quit");
			port_shutdown.write(bot);
			port_shutdown.interrupt();
			port_shutdown.close();
		}
	}

    virtual void run()
    {
		//network checks
		//is yarp server available?
		yarp_found = yarp.checkNetwork();
		if (yarp_found)
		{
			//is output port already open? if not, open it
			if (!yarp.exists((localName+"/battery:o").c_str()))
				port_battery_output.open((localName+"/battery:o").c_str());

		}
		//read battery data
		serial_buff[0]=0;
		int rec = 0;
		do
		{
			rec = pSerial->receiveLine(serial_buff,250);
		}
		while
			(rec>0);

		int len = strlen(serial_buff);
		if (len>0)
		{
			if (verboseEnable)
				fprintf(stderr,"%s", serial_buff);
		}
		
		int pars = 0;
		pars = sscanf (serial_buff, "%*s %d %*s %d", &battery_data.count, &battery_data.raw_voltage);

		if (pars == 2)
		{
			time_t rawtime;
			struct tm * timeinfo;
			time ( &rawtime );
			timeinfo = localtime ( &rawtime );
			battery_data.voltage = double(battery_data.raw_voltage)/1024 * 60;
			battery_data.timestamp = asctime (timeinfo);
			sprintf(log_buffer,"%d %d %f %s", battery_data.count,battery_data.raw_voltage,battery_data.voltage,battery_data.timestamp);
		}
		else
		{
			fprintf(stderr,"error reading battery data: %d\n", pars);
		}

		//send data to yarp output port (if available)
		if (yarp_found)
		{	
			Bottle bot; 
			bot.addString("count");
			bot.addInt(battery_data.count);
			bot.addString("voltage");
			bot.addDouble(battery_data.voltage);
			bot.addString("time");
			bot.addString(battery_data.timestamp);
			port_battery_output.write(bot);
		}

		// The core part: checks on the status of charge of the battery
		{
			//stop_robot("/icub/quit");
			//stop_robot("/ikart/quit");
		}

		//print data to screen
		if (1/*screenEnable*/)
		{
			fprintf(stderr,"%s", log_buffer);
		}
		//save data to file
		if (logEnable)
		{
			fprintf(logFile,"%s", log_buffer);
		}

    }

    virtual void threadRelease()
    {    
		port_battery_output.interrupt();
        port_battery_output.close();

		//close log file
		if (logEnable)
		{
			fclose(logFile);
		}
    }

	void turn_off_control()
	{
	}

};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    //Port        rpcPort;

public:
    CtrlModule() { }

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
        ctrlName=rf.check("ctrlName",Value("iKart")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();

        remoteName=slash+robotName+"/wheels";
        localName=slash+ctrlName;//+"/local/";

        thr=new CtrlThread(10000,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        //rpcPort.open((localName+"/rpc").c_str());
		//attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;
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

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;

    return mod.runModule(rf);
}



