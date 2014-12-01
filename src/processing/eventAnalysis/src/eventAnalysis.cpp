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

#include <iostream>
#include <string>
#include <eventAnalysis.h>
//#include <iCub/emorph/eventCodec.h>

/******************************************************************************/
//EVENT STATISTICS DUMPER
/******************************************************************************/

eventStatisticsDumper::eventStatisticsDumper()
{

    this->setStrict();
    this->moduleName = "Default";
    //outfilename = "/home/aglover/workspace/results/uniquetimestamps.txt";
    prevstamp = 0;
}

void eventStatisticsDumper::setModuleName(std::string name)
{
    this->moduleName = name;
}

void eventStatisticsDumper::setOutputName(std::string name)
{
    this->outfilename = name;
}


bool eventStatisticsDumper::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + ":i";
    BufferedPort<emorph::eBottle>::open( inPortName.c_str() );

    std::cout << "Opening writer: " << outfilename << std::endl;
    fwriter.open(outfilename.c_str());
    if(!fwriter.is_open()) {
        std::cerr << "File did not open at: " << outfilename << std::endl;
    }
    fwriter << "Event Statistics Output File" << std::endl;

    return true;
}

void eventStatisticsDumper::close()
{
    std::cout << "now closing ports..." << std::endl;
    fwriter.close();
    BufferedPort<emorph::eBottle>::close();
    std::cout << "finished closing the read port..." << std::endl;
}

void eventStatisticsDumper::interrupt()
{
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    BufferedPort<emorph::eBottle>::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
}

void eventStatisticsDumper::onRead(emorph::eBottle &bot)
{


    //create event queue
    emorph::eEventQueue q;
    emorph::eEventQueue::iterator qi;
    bot.getAllSorted(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        if ((*qi)->getStamp() < prevstamp)
                fwriter << prevstamp << " " << (*qi)->getStamp() << std::endl;
        prevstamp = (*qi)->getStamp();
    }
}


/******************************************************************************/
//EVENT STATISTICS MODULE
/******************************************************************************/

eventStatisticsModule::eventStatisticsModule()
{

}

bool eventStatisticsModule::configure(yarp::os::ResourceFinder &rf)
{
    if(rf.check("name"))
    {
        name = rf.find("name").asString();
        std::cout << "Module name set to: " << name << std::endl;
    }
    else
    {
        name = "EventStatisticsModule";
        std::cout << "Module name set to default: " << name << std::endl;
    }
    setName(name.c_str());
    esd.setModuleName(name);

    std::string outfilename;
    if(rf.check(("outputFile")))
    {
        outfilename = rf.find(("outputFile")).asString();
        std::cout << "Writing Output to: " << outfilename << std::endl;
    }
    else
    {
        outfilename = "eventAnalysisOutput.txt";
        std::cout << "Default Output: ./" << outfilename << std::endl;
    }
    esd.setOutputName(outfilename);
    esd.open();

    return true;
}

bool eventStatisticsModule::close()
{

    std::cout << "Closing the Event Statistics Module" << std::endl;
    esd.close();
}

bool eventStatisticsModule::updateModule()
{
//    std::cout << name << " " << esd.getBatchedPercentage() << "% batched "
//              << esd.getBatchedCount() << "# more "
//              << esd.getTSCount() << "# TS total "
//              << std::endl;

//    esd.fwriter << esd.getBatchedPercentage() << " "
//                << esd.getBatchedCount() << " "
//                << esd.getTSCount() << " "
//                << std::endl;

    return true;
}



