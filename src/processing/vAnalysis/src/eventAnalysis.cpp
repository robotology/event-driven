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
    dir = "";
    prevstamp = 0;
    bottle_number = 0;
}

bool eventStatisticsDumper::open(std::string moduleName)
{

    //allow input of vBottles
    this->useCallback();
    std::string inPortName = "/" + moduleName + "/vBottle:i";
    BufferedPort<emorph::vBottle>::open( inPortName.c_str() );

    //now initialise and open all our writers
    std::cout << "Opening writers: " << std::endl;
    std::string fname;

    //wrap_stats
    fname = dir + "wrapStats.txt";
    wrap_writer.open(fname.c_str());
    if(!wrap_writer.is_open()) {
        std::cerr << "File did not open at: " << fname << std::endl;
    } else {
        std::cout << fname << " successfully opened" << std::endl;
    }
    wrap_writer << "Event Statistics Output File" << std::endl;
    wrap_writer << "Bottle# Event#/#inBottle PrevStamp CurStamp ..." << std::endl;

    //vBottle size stats
    fname = dir + "bottleSize.txt";
    count_writer.open(fname.c_str());
    if(!count_writer.is_open()) {
        std::cerr << "File did not open at: " << fname << std::endl;
    } else {
        std::cout << fname << " successfully opened" << std::endl;
    }
    count_writer << "Bottle Size | most recent stamp" << std::endl;

    //vBottle size stats
    fname = dir + "stamps.txt";
    stamp_writer.open(fname.c_str());
    if(!stamp_writer.is_open()) {
        std::cerr << "File did not open at: " << fname << std::endl;
    } else {
        std::cout << fname << " successfully opened" << std::endl;
    }

    return true;
}

void eventStatisticsDumper::close()
{
    std::cout << "now closing ports..." << std::endl;
    wrap_writer.close();
    count_writer.close();
    stamp_writer.close();
    BufferedPort<emorph::vBottle>::close();
    std::cout << "finished closing the read port..." << std::endl;
}

void eventStatisticsDumper::onRead(emorph::vBottle &bot)
{

    bottle_number++;
    if(bottle_number % 1000 == 0) {
        //every 1000 bottles put a message so we know the module is
        //actaully running in case of good operation
        wrap_writer << bottle_number << ": ... " << std::endl;
    }
    //std::cout << ". ";

    //create event queue
    emorph::vQueue q = bot.getAll();
    emorph::vQueue::iterator qi, qi2;
    //emorphbot.getAll(q);

    int i = 0, j = 0;

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        i++;
        if ((*qi)->getStamp() < prevstamp) {
            wrap_writer << bottle_number << ": " << i << "/" << q.size() << " : ";
            wrap_writer << prevstamp << " " << (*qi)->getType() <<
                       (*qi)->getStamp() << " ";
            for(j = 0, qi2 = qi; j < 4 && qi2 != q.end(); j++, qi2++)
                wrap_writer << (*qi2)->getType() << (*qi2)->getStamp() << " ";
            wrap_writer << std::endl;

        }
        prevstamp = (*qi)->getStamp();

        emorph::AddressEvent *ae = (*qi)->getAs<emorph::AddressEvent>();
        if(!ae) continue;
        unsigned long int ts = unwrapper(ae->getStamp());
        stamp_writer << ae->getChannel() << " " << ts << " " << 
            (int)ae->getPolarity() << " " << (int)ae->getX() << " " <<
            (int)ae->getY();
            if(i == 1)
                stamp_writer << " 1" << std::endl;
            else
                stamp_writer << " 0" << std::endl;
    }

    count_writer << q.size();
    if(q.size()) count_writer << " | " << q.back()->getStamp();
    count_writer << std::endl;
}


/******************************************************************************/
//EVENT STATISTICS MODULE
/******************************************************************************/

eventStatisticsModule::eventStatisticsModule()
{
    msgflag = true;
}

bool eventStatisticsModule::configure(yarp::os::ResourceFinder &rf)
{

    std::string name = rf.check("name",
                                yarp::os::Value("vAnalysis")).asString();

    std::string dir = rf.check("dir",
                               yarp::os::Value("")).asString();

    runtime = rf.check("runfor",
                       yarp::os::Value(-1)).asDouble();
    starttime = -1;
    setName(name.c_str());

    esd.setDirectory(dir);
    esd.open(name);

    return true;
}

bool eventStatisticsModule::close()
{

    std::cout << "Closing the Event Statistics Module" << std::endl;
    esd.close();
}

bool eventStatisticsModule::updateModule()
{

    if(msgflag && !esd.getInputCount()) {
        std::cout << "Please connect vBottle to port" << std::endl;
        msgflag = false;
    }

    if(!msgflag && esd.getInputCount()) {
        std::cout << "Okay ready to go. Hit Play" << std::endl;
        msgflag = true;
    }

    if(runtime > 0 && esd.getBottleCount()) {
        if(starttime < 0) starttime = timer.now();
        if(timer.now() - starttime > runtime) {
            esd.close();
            return false;
        }
    }

    return true;
}



