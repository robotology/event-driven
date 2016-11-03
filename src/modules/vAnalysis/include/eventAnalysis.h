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
#include <iCub/emorph/all.h>

#include <fstream>


/******************************************************************************/
//EVENT STATISTICS DUMPER
/******************************************************************************/

class eventStatisticsDumper : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    //parameters
    std::string dir;

    //private member variables
    emorph::vtsHelper unwrapper;
    int bottle_number;
    int prevstamp;

    //statistics writers
    std::ofstream wrap_writer;
    std::ofstream count_writer;
    std::ofstream stamp_writer;

public:

    eventStatisticsDumper();

    bool    open(std::string moduleName = "ESD");
    void    onRead(emorph::vBottle &bot);
    void    close();


    void setDirectory(std::string dir) { this->dir = dir; }
    int getBottleCount() { return bottle_number; }







};

/******************************************************************************/
//EVENT STATISTICS MODULE
/******************************************************************************/

class eventStatisticsModule : public yarp::os::RFModule
{

private:

    //our class that does the statistics dumping with an onRead()
    eventStatisticsDumper esd;

    //for connection instructions
    bool msgflag;

    //for timing (if only set to run for a set time period)
    double runtime;
    double starttime;
    yarp::os::Time timer;

public:

    eventStatisticsModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();


    double getPeriod() { return 0.2; }
    bool updateModule();

};



#endif /*__eventAnalysis__*/
