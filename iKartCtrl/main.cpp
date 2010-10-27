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
using namespace yarp::sig;
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
	double              wdt_timeout;

	//movement control variables
	double				linear_speed;
	double				angular_speed;
	double				desired_direction;
	double              pwm_gain;

	//motor variables
	double              FA;
	double              FB;
	double              FC;

protected:
    ResourceFinder      &rf;
    PolyDriver          *laser_driver;
	PolyDriver          *control_board_driver;
    BufferedPort<Bottle>            port_movement_control;
	BufferedPort<yarp::sig::Vector> port_laser_output;

    bool ctrlCompletePose;
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
		ikart_control_type = IKART_CONTROL_OPENLOOP;
		//ikart_control_type = IKART_CONTROL_NONE;
		wdt_timeout = 0.100;

		linear_speed = 1;
		angular_speed = 0;
		desired_direction=0;
		pwm_gain=0;

		FA=0;
		FB=0;
		FC=0;
    }

	void set_ikart_control_type(int type)
	{
		ikart_control_type = type;
	}

	int get_ikart_control_type()
	{
		return ikart_control_type;
	}

    virtual bool threadInit()
    {
		laser_driver=new PolyDriver;
		control_board_driver=new PolyDriver;
		bool ok = true;

		// open the laser scanner driver
		Property laser_options;
		laser_options.fromConfigFile("C:/Software/iCub/main/app/iKart/conf/iKartLaser.ini");
		laser_options.put("device","laserHokuyo");
        if (!laser_driver->open(laser_options))
        {
			fprintf(stderr,"ERROR: cannot open laser driver...\n");
            delete laser_driver;    
            return false;
		}
		//open the interface for the laser
		ok = true;
		ok = ok & laser_driver->view(iLaser);
		if(!ok)
		{
			fprintf(stderr,"ERROR: cannot view the laser interface\nreturning...\n");
			//return false;
		}

        // open the control board driver
        Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote",remoteName.c_str());
        control_board_options.put("local",localName.c_str());
        if (!control_board_driver->open(control_board_options))
        {
			fprintf(stderr,"ERROR: cannot open control board driver...\n");
            //delete control_board_driver;    
            //return false;
        }
        // open the interfaces for the control boards
		ok = true;
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
		port_laser_output.open((localName+"/laser:o").c_str());

		//sets the control mode to the joints
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
		if (ikart_control_type == IKART_CONTROL_SPEED)
		{
			fprintf(stdout,"iKart in speed control mode\n");
			icmd->setVelocityMode(0);
			icmd->setVelocityMode(1);
			icmd->setVelocityMode(2);
		}
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
		iLaser->read(laser_data);
		yarp::sig::Vector &plaser_data=port_laser_output.prepare();
        plaser_data=laser_data;
        //lastStateStamp.update();
		//port_laser_data.setEnvelope(lastStateStamp);
        port_laser_output.write();

		static double wdt_cmd=Time::now();
        if (Bottle *b=port_movement_control.read(false))
        {                
            if (b->size()>=3)
            {                                
				//received movement command
				desired_direction = b->get(0).asDouble();
				linear_speed = b->get(1).asDouble() / 46000 * 1333;
				angular_speed = b->get(2).asDouble() / 46000 * 1333;
				pwm_gain = b->get(3).asDouble() / 65000;
				wdt_cmd = Time::now();
            }
        }

		//watchdog on received commands
        double wdt=Time::now();

		if (ikart_control_type != IKART_CONTROL_NONE)
			{
				if (wdt-wdt_cmd > wdt_timeout)
				{
					//fprintf(stderr,"No commands received in %f ms. Turning off control. \n",wdt_timeout/1000);
					//ikart_control_type = IKART_CONTROL_NONE; 
				}
			}


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
		const int MAX_PWM = 1333; // Maximum motor PWM
		const double ratio = 0.7; // This value must be < 1 
		if (linear_speed  >  MAX_PWM*ratio) linear_speed  = MAX_PWM*ratio;
		if (linear_speed  < -MAX_PWM*ratio) linear_speed  = -MAX_PWM*ratio;
		if (angular_speed >  MAX_PWM*(1-ratio)) angular_speed = MAX_PWM*(1-ratio);
		if (angular_speed < -MAX_PWM*(1-ratio)) angular_speed = -MAX_PWM*(1-ratio);
		if (pwm_gain<0) pwm_gain =0;
		if (pwm_gain>1)	pwm_gain =1;

		//wheel contribution calculation
		FA = linear_speed * cos ((150-desired_direction)/ 180.0 * 3.14159265) + angular_speed;
		FB = linear_speed * cos ((030-desired_direction)/ 180.0 * 3.14159265) + angular_speed;
		FC = linear_speed * cos ((270-desired_direction)/ 180.0 * 3.14159265) + angular_speed;

		if (ikart_control_type == IKART_CONTROL_OPENLOOP)
		{
			iopl->setOutput(0,FA*pwm_gain);
			iopl->setOutput(1,FB*pwm_gain);
			iopl->setOutput(2,FC*pwm_gain);
		}
		else if	(ikart_control_type == IKART_CONTROL_SPEED)
		{
			ivel->velocityMove(0,FA);
			ivel->velocityMove(1,FB);
			ivel->velocityMove(2,FC);
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
        ctrlName=rf.check("ctrlName",Value("iKart")).asString();
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

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;

    return mod.runModule(rf);
}



