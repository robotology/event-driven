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
\defgroup commandGenerator commandGenerator
 
@ingroup icub_tools  
 
@@@TODO
 
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
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/DebugInterfaces.h>

#include <math.h>

#include <iostream>
#include <iomanip>
#include <string>

YARP_DECLARE_DEVICES(icubmod)

#define PRINT_STATUS_PER    0.200     // [s]

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class CommandClass
{
	enum
	{
		COMMAND_SQUARE = 0,
		COMMAND_SINE = 1,
	};

	public:
	CommandClass(double max, double min, double period);
    int	   commandType;
	double current_value;
	double period;
	double min;
	double max;
	int    cycle;

	void   start();
	double update();
	void   stop();

	private:
	bool   enable;
	double time_last;
	double time_start;
	double time_current;
};

CommandClass::CommandClass(double max, double min, double period)
{
	this->max=max;
	this->min=min;
	this->period=period;
	current_value = 0;
	cycle =0;
	time_start = time_current = time_last =Time::now();
}

void CommandClass::start()
{
	time_start = time_current = time_last =Time::now();
	enable = true;
}

void CommandClass::stop()
{
	time_start = time_current = time_last =Time::now();
	enable = false;
}

bool near(double a, double b)
{
	if (abs(a-b)<0.0001) return true;
	else return false;
}

double CommandClass::update()
{
	time_current = Time::now();
	switch (commandType)
	{
		case COMMAND_SQUARE:
		default:
			if (time_current-time_last>period)
			{
				cycle ++;
				if      (near(current_value,max))
						{current_value=min; time_last = time_current;}
				else if (near(current_value,min))
						{current_value=max; time_last = time_current;}
				else    {current_value=max; time_last = time_current;}
			}
		break;
		case COMMAND_SINE:
		
		break;
	}
	return current_value;
}

class CtrlThread: public RateThread
{

private:
	enum
	{
		CONTROL_NONE = 0,
		CONTROL_OPENLOOP = 1,
		CONTROL_SPEED = 2,
	};

	int                 *board_control_modes;
    int					control_type;
	int					command_type;
	double              wdt_timeout;
	double              command;
	int                 cycle;
	double              measure_speed;
	double              measure_pwm;
	int                 joint;
	CommandClass        *commandProc;
    FILE *              pFile;

protected:
    ResourceFinder      &rf;
	PolyDriver          *control_board_driver;
	PolyDriver          *debug_driver;
    BufferedPort<Bottle>            port_movement_control;
	BufferedPort<yarp::sig::Vector> port_output1;

    string remoteName;
    string localName;

	IPidControl       *ipid;
	IPositionControl  *ipos;
	IVelocityControl  *ivel;
	IEncoders         *ienc;
	IAmplifierControl *iamp;
	IOpenLoopControl  *iopl;
	IControlMode      *icmd;
	IDebugInterface   *idbg;

    double t0;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) 
	{
		board_control_modes = new int [3];
		control_type = CONTROL_SPEED;
		//control_type = CONTROL_OPENLOOP;
		//control_type = CONTROL_NONE;
		wdt_timeout = 0.100;
		command=0;
		joint=2;
		//commandProc = new CommandClass(0.5,-0.5,1.0);
		if (control_type == CONTROL_SPEED)
		{
			//slow speed
			//commandProc = new CommandClass(0,2,5.0);
			//fast speed
			commandProc = new CommandClass(0,80,4.0);
		}
    }

	void set_control_type(int type)
	{
		control_type = type;
	}

	int get_control_type()
	{
		return control_type;
	}

    virtual bool threadInit()
    {

        pFile = fopen ("command_log.txt","w");

		control_board_driver=new PolyDriver;
		bool ok = true;

        // open the control board driver
        Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote",remoteName.c_str());
        control_board_options.put("local",localName.c_str());
        if (!control_board_driver->open(control_board_options))
        {
			fprintf(stderr,"ERROR: cannot open control board driver...\n");
            delete control_board_driver;    
            return false;
        }
        // open the interfaces for the control boards
		ok = true;
		ok = ok & control_board_driver->view(ivel);
		ok = ok & control_board_driver->view(ienc);
		ok = ok & control_board_driver->view(iopl);
		ok = ok & control_board_driver->view(ipid);
		ok = ok & control_board_driver->view(iamp);
		ok = ok & control_board_driver->view(icmd);
		ok = ok & control_board_driver->view(ipos);
		if(!ok)
		{
			fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...\n");
			//return false;
		}

		// open the debug driver
		Property debug_options;
		localName+="/debug/";
        debug_options.put("local", localName.c_str());	
        debug_options.put("device", "debugInterfaceClient");
		debug_options.put("remote", remoteName.c_str());
		debug_driver = new PolyDriver(debug_options);
		if(debug_driver->isValid() == false)
        {
			fprintf(stderr,"ERROR: cannot open debugInterfaceClient...\n");
            delete debug_driver;    
            return false;
        }
        // open the interfaces for the control boards
		ok = true;
		ok = ok & debug_driver->view(idbg);
		if(!ok)
		{
			fprintf(stderr,"ERROR: idbg interface has not been viewed\nreturning...\n");
			return false;
		}

        // open ports
        port_movement_control.open((localName+"/control:i").c_str());
		port_output1.open((localName+"/out1:o").c_str());

		/*
		//sets the control mode to the joints
		if (control_type == CONTROL_OPENLOOP)
		{
			fprintf(stdout,"Using openloop control mode\n");
			icmd->setOpenLoopMode(joint);
			iopl->setOutput(joint,0);
		}
		if (control_type == CONTROL_SPEED)
		{
			fprintf(stdout,"Using speed control mode\n");
			icmd->setVelocityMode(joint);
		}
		*/
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            cout<<"Thread started successfully"<<endl;
        else
            cout<<"Thread did not start"<<endl;

        t0=Time::now();
    }

    virtual void run()
    {
		//read laser data
		yarp::sig::Vector laser_data;
		yarp::sig::Vector &plaser_data=port_output1.prepare();
        plaser_data=laser_data;
        //lastStateStamp.update();
		//port_laser_data.setEnvelope(lastStateStamp);
        port_output1.write();

		static double wdt_cmd=Time::now();
        if (Bottle *b=port_movement_control.read(false))
        {                
            if (b->size()>=3)
            {                                
				//received something
            }
        }

		//watchdog on received commands
        double wdt=Time::now();

		if (control_type != CONTROL_NONE)
			{
				if (wdt-wdt_cmd > wdt_timeout)
				{
					//fprintf(stderr,"No commands received in %f ms. Turning off control. \n",wdt_timeout/1000);
					//ikart_control_type = IKART_CONTROL_NONE; 
				}
			}

		double t=Time::now();
		static double t1=Time::now();
		static int count = 0;

		if (control_type == CONTROL_OPENLOOP)
		{
			//iopl->setOutput(0,command);
		}
		else if	(control_type == CONTROL_SPEED)
		{
			command = commandProc->update();
			cycle = commandProc->cycle;
						ivel->velocityMove(0,command);
						ivel->velocityMove(1,command);
						ivel->velocityMove(2,command);
			ienc->getEncoderSpeed(joint,&measure_speed);
			ipid->getOutput(joint,&measure_pwm);

			fprintf (stdout,"%4d cycle: %3d t: %+8.3f cmd: %+8.2f err: %+8.2f sp:%+8.2f pwm:%+8.2f \n",
						     count,
							 cycle,
							 t-t1,
							 command*2,
							 command*2-measure_speed,
							 measure_speed,
							 measure_pwm);
			/*fprintf (pFile, "%d %d %+.3f %+.2f %+.2f %+.2f %+.2f %+.2f\n",
							 count,
							 cycle,
							 t-t1,
							 command*2,
							 command*2-measure_speed,
							 measure_speed,
							 measure_pwm);
			*/
		}
		else if (control_type == CONTROL_NONE)
		{
			//iopl->setOutput(0,0);
			//iopl->setOutput(1,0);
			//iopl->setOutput(2,0);
		}


		t1=t;
		count++;

		printStatus();
    }

    virtual void threadRelease()
    {    
		delete control_board_driver;

        port_movement_control.interrupt();
        port_movement_control.close();
		port_output1.interrupt();
        port_output1.close();
    }

	void turn_off_control()
	{
		set_control_type (CONTROL_NONE);
	}

    void printStatus()
    {
        double t=Time::now();

        if (t-t0>=PRINT_STATUS_PER)
        {
			//fprintf (stdout,"alive, time: %f\n",t-t0);
			//fprintf (stdout,"cycle: %d time: %+.3f command: %+.2f measure %+.2f error: %+.2f\n",cycle, t-t0, command, measure, command-measure);
			//fprintf (stdout,"\n");
            t0=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    Port        rpcPort;

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
        ctrlName=rf.check("ctrlName",Value("commandGenerator")).asString();
 
		//robotName=rf.check("robot",Value("icubV2")).asString();
        //remoteName=slash+robotName+"/right_arm";

		robotName=rf.check("robot",Value("ikart")).asString();
        remoteName=slash+robotName+"/wheels";


		localName=slash+ctrlName;//+"/local/";

        thr=new CtrlThread(10,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
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
        thr->stop();
        delete thr;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
	YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\tNo options at the moment"<< endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
	{
		fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
	}

    CtrlModule mod;

    return mod.runModule(rf);
}
