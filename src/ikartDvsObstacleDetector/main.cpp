/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

#include "ikartDvsObstacleDetector.h"
#include <math.h>

class iKartPlannerModule : public yarp::os::RFModule
{
protected:
    yarp::os::Port              rpcPort;

public:
	obstacleDetectorThread     *obstacleThread;

    iKartPlannerModule()
    {
        obstacleThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        Property p;
        ConstString configFile = rf.findFile("from");
        if (configFile!="") p.fromConfigFile(configFile.c_str());
		
		int period = 20; //ms
        obstacleThread = new obstacleDetectorThread(period,rf,p);

        if (!obstacleThread->start())
        {
            delete obstacleThread;
            return false;
        }

        rpcPort.open("/ikartDvsObstacleDetector/rpc:i");
        attach(rpcPort);
        //attachTerminal();

        return true;
    }

    virtual bool interruptModule()
    {
        rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        //obstacleThread->shutdown();
        obstacleThread->stop();
        delete obstacleThread;
        obstacleThread=NULL;

        return true;
    }

    virtual double getPeriod()
    { 
        return 1.0; 
    }
    
    virtual bool updateModule()
    { 
        if (isStopping())
        {
            obstacleThread->stop();   
            return false;
        }
        
        return true; 
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
		reply.clear(); 
		
        if (command.get(0).asString()=="quit") return false;     

		else if (command.get(0).asString()=="help")
		{
			reply.addString("Available commands are:");
			reply.addString("quit");
		}
		else if (command.get(0).asString()=="s")
		{
			double f = command.get(1).asDouble();
			double ang= command.get(2).asDouble();
			double h = command.get(3).asDouble();
			obstacleThread->set_model_params(f,ang,h);
			reply.addString("Params loaded");
		}
		else if (command.get(0).asString()=="g")
		{
			char buff[255];
			double f, ang, h;
			obstacleThread->get_model_params(f, ang, h);
			sprintf (buff, "f: %f ang: %f h: %f\n",f, ang, h);
			reply.addString(buff);
		}
        else
        {
            reply.addString("Unknown command.");
        }
		
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"ERROR: check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("ikartDvsObstacleDetector.ini");		   //overridden by --from parameter
    rf.setDefaultContext("ikartDvsObstacleDetector/conf");             //overridden by --context parameter
    rf.configure("ICUB_ROOT",argc,argv);
    
    iKartPlannerModule iKartPlanner;

    return iKartPlanner.runModule(rf);
}

 
