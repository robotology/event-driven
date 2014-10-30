/*
 * Copyright (C) 2014 Istituto Italiano di Tecnologia
 * Author: Arren Glover
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

#ifndef __eventAnalysis__
#define __eventAnalysis__

#include <yarp/os/all.h>
#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/eventCodec.h>

#include <fstream>


/******************************************************************************/
//EVENT STATISTICS DUMPER
/******************************************************************************/

class eventStatisticsDumper : public yarp::os::BufferedPort<eventBottle>
{
public:

    eventStatisticsDumper();
    void setModuleName(std::string name);

    bool    open();
    void    close();
    void    onRead(eventBottle &bot);
    void    interrupt();

private:

    double eventsPerTS;
    unsigned long ts;
    unsigned long total;
    double batched;
    std::string outfilename;
    std::ofstream fwriter;

    std::string     moduleName;         //string containing module name
    std::string     inPortName;        	//speech input port

};

/******************************************************************************/
//EVENT STATISTICS MODULE
/******************************************************************************/

class eventStatisticsModule : public yarp::os::RFModule
{

private:
    std::string name;
    eventStatisticsDumper esd;

public:

    eventStatisticsModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();


    double getPeriod() { return 1.0; }
    bool updateModule() {
        std::cout << "BLAH: " << name << std::endl;
        return true; }

};



#endif /*__eventAnalysis__*/
