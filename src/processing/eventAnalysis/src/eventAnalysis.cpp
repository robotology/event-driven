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
    eventsPerTS = 1;
    this->setStrict();
    this->moduleName = "Default";
    outfilename = "/home/aglover/workspace/results/uniquetimestamps.txt";
    fwriter.open(outfilename.c_str());
    if(!fwriter.is_open()) {
        std::cerr << "File did not open at: " << outfilename << std::endl;
    }
    ts = 0;
    total = 0;
    batched = 0;
}

void eventStatisticsDumper::setModuleName(std::string name)
{
    this->moduleName = name;
}


bool eventStatisticsDumper::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + "/etb:i";
    BufferedPort<eventBottle >::open( inPortName.c_str() );

    return true;
}

void eventStatisticsDumper::close()
{
    std::cout << "now closing ports..." << std::endl;
    fwriter << batched / total << std::endl;
    fwriter.close();
    BufferedPort<eventBottle >::close();
    std::cout << "finished closing the read port..." << std::endl;
}

void eventStatisticsDumper::interrupt()
{
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    BufferedPort<eventBottle >::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
}

void eventStatisticsDumper::onRead(eventBottle &bot)
{


    //create event queue
    emorph::ecodec::eEventQueue q;


    if(emorph::ecodec::eEvent::decode(*bot.get_packet(),q))
    {
        int size = q.size();
        //fprintf(stdout, "eventBottle size: %d\n",size);
        for (int e = 0; e < size; e++)
        {
            if(q[e] != 0)
            {
                if(q[e]->getType()=="TS") //identify the type of the packet (Time Stamp)
                {
                    //write the number of events per timestamp to a file
                    //std::cout << eventsPerTS << std::endl;
                    emorph::ecodec::TimeStamp* ptr =
                            dynamic_cast<emorph::ecodec::TimeStamp*>(q[e]);
                    unsigned long tstemp = (unsigned long) ptr->getStamp();
                    total++;
                    if(ts == tstemp) {
                            eventsPerTS++;
                    } else {
                        if(eventsPerTS > 1) batched++;
                        fwriter << eventsPerTS << std::endl;
                        eventsPerTS = 1;
                    }
                    ts = tstemp;

                }


            }
        }

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

    esd.setModuleName(std::string("StatisticsDumper"));
    esd.open();

    return true;
}

bool eventStatisticsModule::close()
{

    std::cout << "Closing the Event Statistics Module" << std::endl;
    esd.close();
}



