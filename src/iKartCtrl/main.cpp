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
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <math.h>

#include <iostream>
#include <iomanip>
#include <string>

#define PRINT_STATUS_PER    0.5     // [s]

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;


class CtrlThread: public RateThread
{

private:
	enum
	{
		IKART_CONTROL_NONE = 0,
		IKART_CONTROL_OPENLOOP = 1,
		IKART_CONTROL_SPEED = 2
	};
		 
	int                 *board_control_modes;
    int					ikart_control_type;
	double              wdt_mov_timeout;
	double              wdt_joy_timeout;
	int                 timeout_counter;
	int                 joystick_counter;

	//movement control variables
	double				linear_speed;
	double				angular_speed;
	double				desired_direction;
	double              pwm_gain;

	//motor variables
	double              FA;
	double              FB;
	double              FC;

	//laser vaiables
	bool                laser_enabled;

protected:
    ResourceFinder      &rf;
    PolyDriver          *laser_driver;
	PolyDriver          *control_board_driver;
    BufferedPort<Bottle>            port_movement_control;
	BufferedPort<Bottle>            port_joystick_control;
	BufferedPort<yarp::sig::Vector> port_laser_output;

    bool   filter_enabled;
	bool   motors_enabled;
    string remoteName;
    string localName;

	IPidControl       *ipid;
	IVelocityControl  *ivel;
	IEncoders         *ienc;
	IAmplifierControl *iamp;
	IOpenLoopControl  *iopl;
	IControlMode      *icmd;
	IAnalogSensor     *iLaser;

    double t0;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) 
	{
		board_control_modes = new int [3];
		ikart_control_type  = IKART_CONTROL_NONE;
		wdt_mov_timeout     = 0.200;
		wdt_joy_timeout     = 0.200;
		timeout_counter     = 0;
		joystick_counter    = 0;

		linear_speed        = 1;
		angular_speed       = 0;
		desired_direction   = 0;
		pwm_gain            = 0;

		FA = 0;
		FB = 0;
		FC = 0;

		filter_enabled = true;
		laser_enabled  = true;
		motors_enabled = true;
    }

	void set_ikart_control_type(int type)
	{
		ikart_control_type = type;
		
		if (ikart_control_type == IKART_CONTROL_OPENLOOP)
		{
			fprintf(stdout,"iKart in openloop control mode\n");
			icmd->setOpenLoopMode(0);
			icmd->setOpenLoopMode(1);
			icmd->setOpenLoopMode(2);
			iopl->setOutput(0,0);
			iopl->setOutput(1,0);
			iopl->setOutput(2,0);
		}
		else if (ikart_control_type == IKART_CONTROL_SPEED)
		{
			fprintf(stdout,"iKart in speed control mode\n");
			icmd->setVelocityMode(0);
			icmd->setVelocityMode(1);
			icmd->setVelocityMode(2);
		}
		else
		{
			fprintf(stdout,"invalid iKart control mode\n");
		}
	}

	int get_ikart_control_type()
	{
		return ikart_control_type;
	}

    virtual bool threadInit()
    {
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
			ConstString control_type = iKartCtrl_options.check("control_mode",Value("none"),"type of control for the wheels").asString();
			if      (control_type == "none")     ikart_control_type = IKART_CONTROL_NONE;
			else if (control_type == "speed")    ikart_control_type = IKART_CONTROL_SPEED;
			else if (control_type == "openloop") ikart_control_type = IKART_CONTROL_OPENLOOP;
			else
			{
				fprintf(stderr,"Error: unknown type of control required: %s. Closing...\n",control_type.c_str());
				return false;
			}
		}

		if (rf.check("no_filter"))
		{
			printf("\n'no_filter' option found. Turning off PWM filter.\n");
			filter_enabled=false;
		}

		if (iKartCtrl_options.check("laser")==false ||
			rf.check("no_laser")==true)
		{
			printf("\nLaser configuration not specified. Turning off laser.\n");
			laser_enabled=false;
		}
		else
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
			}
		}

		if (rf.check("no_motors"))
		{
			printf("\n'no_motors' option found. Skipping motor control part.\n");
			motors_enabled=false;
		}
		if (!motors_enabled) return true;

        // open the control board driver
		control_board_driver=new PolyDriver;
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
		bool ok = true;
		ok = ok & control_board_driver->view(ivel);
		ok = ok & control_board_driver->view(ienc);
		ok = ok & control_board_driver->view(iopl);
		ok = ok & control_board_driver->view(ipid);
		ok = ok & control_board_driver->view(iamp);
		ok = ok & control_board_driver->view(icmd);
		if(!ok)
		{
			fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...\n");
			//return false;
		}
        // open ports
        port_movement_control.open((localName+"/control:i").c_str());
		port_joystick_control.open((localName+"/joystick:i").c_str());
		port_laser_output.open((localName+"/laser:o").c_str());

		//set the control type
		if (!rf.check("no_start"))
		{
			printf("starting motors...");
			iamp->enableAmp(0);
			iamp->enableAmp(1);
			iamp->enableAmp(2);			
		}
		set_ikart_control_type(IKART_CONTROL_SPEED);

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Thread started successfully\n");
        else
            printf("Thread did not start\n");

        t0=Time::now();
    }

    double lp_filter(double input, int i)
    {
	   //This is a butterworth low pass first order, with a cut off freqency of 1Hz
	   //It must be used with a sampling frequency of 50Hz (20ms)
       static double xv[2][10], yv[2][10];
       xv[0][i] = xv[1][i]; 
       xv[1][i] = input /1.689454484e+01;
       yv[0][i] = yv[1][i]; 
       yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.8816185924 * yv[0][i]);
       return yv[1][i];
    }

    virtual void run()
    {
		//read laser data
		if (laser_enabled)
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
		
		// if motors are disabled the loop finishes here.
		if (!motors_enabled)
		{
			return;
		}

		static double wdt_mov_cmd=Time::now();
		static double wdt_joy_cmd=Time::now();		

		if (Bottle *b = port_joystick_control.read(false))
        {                
            if (b->get(0).asInt()==1)
            {                                
				//received a joystick command.
				desired_direction = b->get(1).asDouble();
				linear_speed = b->get(2).asDouble();
				angular_speed = b->get(3).asDouble();
				pwm_gain = b->get(4).asDouble();
				wdt_joy_cmd = Time::now();
				//Joystick commands have higher priorty respect to movement commands.
				//this make the joystick to take control for 100*20 ms
				if (pwm_gain>10) joystick_counter = 100;
            }
        }
		if (Bottle *b = port_movement_control.read(false))
		{                
			if (b->get(0).asInt()==1)
			{                                
				//received a movement command
				if (joystick_counter==0)
				{	
					//execute the command only if the joystick is not controlling!
					desired_direction = b->get(1).asDouble();
					linear_speed = b->get(2).asDouble();
					angular_speed = b->get(3).asDouble();
					pwm_gain = b->get(4).asDouble();
				}
				wdt_mov_cmd = Time::now();
			}
		}
		
		//watchdog on received commands
        static double wdt_old=Time::now();
		double wdt=Time::now();
		//fprintf(stderr,"period: %f\n", wdt-wdt_old);
		if (wdt-wdt_old > 0.040) { timeout_counter++;  }
		wdt_old=wdt;
		if (joystick_counter>0)  { joystick_counter--; }

	//	icmd->getControlModes(board_control_modes);
	/*	for (int i=0; i<3; i++)
			if (board_control_modes[i]==VOCAB_CM_IDLE)
			{
				fprintf (stderr,"One motor is in idle state. Turning off control.");
				turn_off_control();
				break;
			}
	*/

		//saturators
		int MAX_VALUE = 0;
		if (ikart_control_type == IKART_CONTROL_OPENLOOP)
		{
			MAX_VALUE = 1333; // Maximum joint PWM
		}
		else if	(ikart_control_type == IKART_CONTROL_SPEED)
		{
			MAX_VALUE = 200; // Maximum joint speed (deg/s)
		}
		
		linear_speed = linear_speed / 46000 * MAX_VALUE;
		angular_speed = angular_speed / 46000 * MAX_VALUE;
		pwm_gain = pwm_gain / 65000 * 1.0;
		if (ikart_control_type == IKART_CONTROL_OPENLOOP)
		{
			const double ratio = 0.7; // This value must be < 1 
			if (linear_speed  >  MAX_VALUE*ratio) linear_speed  = MAX_VALUE*ratio;
			if (linear_speed  < -MAX_VALUE*ratio) linear_speed  = -MAX_VALUE*ratio;
			if (angular_speed >  MAX_VALUE*(1-ratio)) angular_speed = MAX_VALUE*(1-ratio);
			if (angular_speed < -MAX_VALUE*(1-ratio)) angular_speed = -MAX_VALUE*(1-ratio);
		}
		if (pwm_gain<0) pwm_gain = 0;
		if (pwm_gain>1)	pwm_gain = 1;

		//wheel contribution calculation
		FA = linear_speed * cos ((150-desired_direction)/ 180.0 * 3.14159265) + angular_speed;
		FB = linear_speed * cos ((030-desired_direction)/ 180.0 * 3.14159265) + angular_speed;
		FC = linear_speed * cos ((270-desired_direction)/ 180.0 * 3.14159265) + angular_speed;
		FA *= pwm_gain;
		FB *= pwm_gain;
		FC *= pwm_gain;
        
		//Use a low pass filter to obtain smooth control
		if (filter_enabled)
		{
			FA  = lp_filter(FA,0);
			FB  = lp_filter(FB,1);
			FC  = lp_filter(FC,2);
		}

		//Apply the commands
		if (ikart_control_type == IKART_CONTROL_OPENLOOP)
		{
			iopl->setOutput(0,-FA);
			iopl->setOutput(1,-FB);
			iopl->setOutput(2,-FC);
		}
		else if	(ikart_control_type == IKART_CONTROL_SPEED)
		{
			ivel->velocityMove(0,-FA);
			ivel->velocityMove(1,-FB);
			ivel->velocityMove(2,-FC);
		}
		else if (ikart_control_type == IKART_CONTROL_NONE)
		{
			//iopl->setOutput(0,0);
			//iopl->setOutput(1,0);
			//iopl->setOutput(2,0);
		}

        printStatus();
    }

    virtual void threadRelease()
    {    
        delete laser_driver;
		delete control_board_driver;

        port_movement_control.interrupt();
        port_movement_control.close();
		port_laser_output.interrupt();
        port_laser_output.close();
		port_joystick_control.interrupt();
        port_joystick_control.close();
    }

	void turn_off_control()
	{
		set_ikart_control_type (IKART_CONTROL_NONE);
	}

    void printStatus()
    {
        double t=Time::now();

        if (t-t0>=PRINT_STATUS_PER)
        {
			//fprintf (stdout,"alive, time: %f\n",t-t0);
			fprintf (stdout,"Timeouts: %d\n",timeout_counter);
			if (joystick_counter>0) 
				fprintf (stdout,"Under joystick control (%d)\n",joystick_counter);
			fprintf (stdout,"FA: %+.1f\n",FA);
			fprintf (stdout,"FB: %+.1f\n",FB);
			fprintf (stdout,"FC: %+.1f\n",FC);
			fprintf (stdout,"\n");
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
        ctrlName=rf.check("ctrlName",Value("ikart")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();

        remoteName=slash+robotName+"/wheels";
        localName=slash+ctrlName;//+"/local/";

        thr=new CtrlThread(20,rf,remoteName,localName);
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
		printf("'laser <filename>' starts the laser with the specified configuration file.\n");
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



