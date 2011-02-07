/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Scalzo
 * email:  alessandro.scalzo@iit.it
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
\defgroup iKartNav iKartNav
 
@ingroup icub_module
 
A reactive navigation module based on laser range finder.
 
Copyright (C) 2011 RobotCub Consortium
 
Author: Alessandro Scalzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module implements a real case of Lumelsky's Bug 1 algorithm, where obstacles are detected by a laser range finder. 
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
The only used parameter is the name of the configuration file. The configuration file name can be specified using --from \e file 
You can also use the \e --context option to change the current context (e.g. the directory where to search the configuration file).
 
\section portsa_sec Ports Accessed
- /ikart/laser:o laser range finder data as yarp::os::Vector
- /ikart/control:i heading, linear and angular speed command to iKartCtrl module as yarp::os::Bottle, with the format: 
<heading [deg]> <speed [m/s]> <angular speed [deg/s]> <pwm gain [0.0-1.0]>
 
\section portsc_sec Ports Created 
- /ikartnav/target:i accepts target command as yarp::os::Bottle with the format: set_target <heading [deg]> <distance [m]>
- /ikartnav/control:o heading, linear and angular speed command to iKartCtrl module as yarp::os::Bottle, with the format: 
<heading [deg]> <speed [m/s]> <angular speed [deg/s]> <pwm gain [0.0-1.0]>

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
A description of the available configuration options can be found in the example files located under /app/iKart/conf/iKartNav.ini
 
\section tested_os_sec Tested OS
Windows, Linux

\author Alessandro Scalzo
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

#include "Vec2D.h"
#include "NavThread.h"
#include "LaserThread.h"

class iKartNavModule : public yarp::os::RFModule
{
protected:
    LaserThread *mLaserThread;
    NavThread   *mNavThread;
    yarp::os::Port mHandlerPort;

public:
    iKartNavModule()
    {
        mNavThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        int period=int(1000.0*rf.check("period",yarp::os::Value(0.02)).asDouble());
        
        mNavThread=new NavThread(period,&rf);

        if (!mNavThread->start())
        {
            delete mNavThread;
            return false;
        }

        mLaserThread=new LaserThread(&rf,mNavThread);

        if (!mLaserThread->start())
        {
            mNavThread->stop();

            delete mNavThread;
            delete mLaserThread;

            return false;
        }

        std::string local=rf.check("local",yarp::os::Value("/ikartnav")).asString().c_str();
        mHandlerPort.open(local.c_str());
        attach(mHandlerPort);
        attachTerminal();

        return true;
    }

    virtual bool interruptModule()
    {
        mHandlerPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        mLaserThread->stop();
        delete mLaserThread;
        mLaserThread=NULL;

        mNavThread->stop();
        delete mNavThread;
        mNavThread=NULL;

        mHandlerPort.interrupt();
        mHandlerPort.close();

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
            mLaserThread->stop();
            mNavThread->stop();
            
            return false;
        }
        
        return true; 
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        if (command.get(0).asString()=="quit") return false;     

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
    rf.setDefaultConfigFile("iKartNav.ini");		   //overridden by --from parameter
    rf.setDefaultContext("../../app/iKart/conf");      //overridden by --context parameter
    rf.configure("ICUB_ROOT",argc,argv);
    
    iKartNavModule iKartNav;

    return iKartNav.runModule(rf);
}
 