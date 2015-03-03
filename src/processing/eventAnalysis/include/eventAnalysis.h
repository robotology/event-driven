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
public:

    eventStatisticsDumper();
    bool    open(std::string moduleName = "ESD");
    void    close();
    void    onRead(emorph::vBottle &bot);
    void    interrupt();

    void setDirectory(std::string dir) {
        this->dir = dir;
    }


    std::ofstream wrap_writer;
    std::ofstream count_writer;
    std::ofstream stamp_writer;

private:

    emorph::vtsHelper unwrapper;


    std::string dir;

    int bottle_number;

    std::string outfilename;
    int prevstamp;

};

/******************************************************************************/
//EVENT STATISTICS MODULE
/******************************************************************************/

class eventStatisticsModule : public yarp::os::RFModule
{

private:

    eventStatisticsDumper esd;
    bool msgflag;

public:

    eventStatisticsModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();


    double getPeriod() { return 1.0; }
    bool updateModule();

};



#endif /*__eventAnalysis__*/
