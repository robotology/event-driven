/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include "module.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2.0)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* create the module */
    module instance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vSkinInterface.ini" );
    rf.configure( argc, argv );

    return instance.runModule(rf);
}


/*////////////////////////////////////////////////////////////////////////////*/
//vSkinInterfaceModule
/*////////////////////////////////////////////////////////////////////////////*/
bool module::configure(yarp::os::ResourceFinder &rf)
{
    //administrative options
    setName((rf.check("name", yarp::os::Value("/skinInterface")).asString()).c_str());

    if(!skinevents_in.open(getName() + "/SKE:i")) {
        yError() << "Could not open events port";
        return false;
    }

    if(!skinsamples_in.open(getName() + "/SKS:i")) {
        yError() << "Could not open samples port";
        return false;
    }

    if(!scope_out.open(getName() + "scope:o")) {
        yError() << "Could not open scope port";
        return false;
    }

    return true;

}

/******************************************************************************/
bool module::interruptModule()
{

    skinevents_in.close();
    skinsamples_in.close();
    scope_out.close();

    return true;
}

/******************************************************************************/
bool module::close()
{
    skinevents_in.close();
    skinsamples_in.close();
    scope_out.close();

    return true;
}

/******************************************************************************/
bool module::updateModule()
{
    Bottle rates;
    yarp::os::Stamp yarp_stamp;

    unsigned int eventpackets = skinevents_in.queryunprocessed();
    if(eventpackets)
        rates.addInt32(skinevents_in.queryRate());
    else
        rates.addInt32(0);
    for(unsigned int i = 0; i < eventpackets; i++)
        const std::vector<ev::SkinEvent> *q = skinevents_in.read(yarp_stamp);

    unsigned int samplepackets = skinsamples_in.queryunprocessed();
    if(samplepackets)
        rates.addInt32(skinsamples_in.queryRate());
    else
        rates.addInt32(0);
    for(unsigned int i = 0; i < samplepackets; i++)
        const std::vector<ev::SkinSample> *q = skinsamples_in.read(yarp_stamp);

    scope_out.prepare() = rates;
    scope_out.write();

    return true;
}

/******************************************************************************/
double module::getPeriod()
{
    return 0.2;
}


