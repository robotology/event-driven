/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#include "spinterface.h"
#include <map>
#include <list>
#include <math.h>

using namespace ev;

/******************************************************************************/
// vSpinInterface
/******************************************************************************/
bool vSpinInterface::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("spinterface")).asString();
    setName(moduleName.c_str());


    /* create the thread and pass pointers to the module parameters */
    spinnio::EIEIOSender   *spinSender;
    spinnio::EIEIOReceiver *spinReceiver;

    // Create Spin sender and receiver objects


    initSpin(17895, 12346, "192.168.240.254",
                       "/home/ubuntu/VVV/AttentionNetwork/application_generated_data_files/latest/input_output_database.db",
             &spinReceiver, &spinSender);



//    int spinPort = 17895;
//    int sendPort = 12346;
//    std::string ip = "192.168.1.1";
//    std::string databasefile =  "/home/ubuntu/VVV/TestNetwork/application_generated_data_files/latest/input_output_database.db";

//    spinReceiver  = new spinnio::EIEIOReceiver(spinPort,(char*)ip.c_str(), true, (char*)databasefile.c_str());

//    std::map<int,int> *keymap = spinReceiver->getIDKeyMap();

//    spinSender = new spinnio::EIEIOSender(sendPort,(char*)ip.c_str(), keymap);

//    spinReceiver->start();
//    spinSender->start();
//    spinSender->enableSendQueue();


    // Start the output manager thread
    bool oSuccess = outputManager.initThread(moduleName, spinReceiver);
    outputManager.start();
    outputManager.run();

    inputManager.attachEIEIOSender(spinSender);
    bool iSuccess = inputManager.open(moduleName);



    return oSuccess && iSuccess;

}

/******************************************************************************/
void vSpinInterface::initSpin(int spinPort, int sendPort, std::string ip,
              std::string databasefile, spinnio::EIEIOReceiver **eir,
                              spinnio::EIEIOSender **eis)
{
    (*eir)  = new spinnio::EIEIOReceiver(spinPort,(char*)ip.c_str(), true,
                                      (char*)databasefile.c_str());

    std::map<int,int> *keymap = (*eir)->getIDKeyMap();

    (*eis) = new spinnio::EIEIOSender(sendPort,(char*)ip.c_str(), keymap);

    (*eir)->start();
    (*eis)->start();
    (*eis)->enableSendQueue();

}

/******************************************************************************/
bool vSpinInterface::interruptModule()
{
    outputManager.stop();
    inputManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/******************************************************************************/
bool vSpinInterface::close()
{
    outputManager.threadRelease();
    inputManager.close();
    yarp::os::RFModule::close();
    return true;
}

/******************************************************************************/
bool vSpinInterface::updateModule()
{
    return true;
}

/******************************************************************************/
double vSpinInterface::getPeriod()
{
    return 1;
}

/******************************************************************************/
// YARP - SPIN - INPUT
/******************************************************************************/
YARPspinI::YARPspinI()
{

    //here we should initialise the module
    height = 128;
    width = 128;
    downsamplefactor = 2;
    //eventsin.open("eventssenttospinnaker.txt");

}

/******************************************************************************/
bool YARPspinI::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    return yarp::os::BufferedPort<ev::vBottle>::open(inPortName);
}

/******************************************************************************/
void YARPspinI::close()
{
    spinSender->closeSendSocket();
    delete spinSender;
    yarp::os::BufferedPort<ev::vBottle>::close();

    //remember to also deallocate any memory allocated by this class
}

/******************************************************************************/
void YARPspinI::interrupt()
{
    yarp::os::BufferedPort<ev::vBottle>::interrupt();

}

/**********************************************************/
void YARPspinI::attachEIEIOSender(spinnio::EIEIOSender* spinSenderPtr)
{
   spinSender = spinSenderPtr;
}

/**********************************************************/
void YARPspinI::onRead(ev::vBottle &bot)
{
    //create event queue
    ev::vQueue q = bot.get<AE>();

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        auto v = is_event<AddressEvent>(*qi);
        if(v->getChannel()) continue;

        int neuronID = (v->y >> downsamplefactor) *
                (width / pow(downsamplefactor, 2.0)) +
                (v->x >> downsamplefactor);

        //eventsin << (int)(v->x) << " " << (int)(v->y) << " " << neuronID << std::endl;
        spinSender->addSpikeToSendQueue(neuronID);

    }

}

/******************************************************************************/
// YARP - SPIN - OUTPUT
/******************************************************************************/
YARPspinO::YARPspinO() : yarp::os::RateThread(1)
{

  // Do we need anything here?
    width = 128;
    height = 128;
    downsamplefactor = 2;

}

/**********************************************************/

bool YARPspinO::initThread(std::string moduleName, spinnio::EIEIOReceiver *spinReceiverPtr)
{
  // Output thread initialisation
    spinReceiver = spinReceiverPtr;
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    return vBottleOut.open(outPortName);
}


/**********************************************************/

void YARPspinO::run()
{

    //get the data
    int recvQueueSize = spinReceiver->getRecvQueueSize();
    if (recvQueueSize <= 0) return;
    //std::cout << "Size: " << recvQueueSize << std::endl;

    //there is data to process so prepare out outbottle port
    ev::vBottle &outbottle = vBottleOut.prepare();
    outbottle.clear();

    auto ae = make_event<AddressEvent>();

    //convert the data to readable packets
    std::list<std::pair<int, int> > spikepacket =
            spinReceiver->getNextSpikePacket();

    //iterate over all spikes and add them to the bottle
    std::list<std::pair<int, int> >::iterator i;

    int rowsize = width >> downsamplefactor;
    for(i = spikepacket.begin(); i != spikepacket.end(); i++) {
        std::pair<int,int> spikeEvent = *i;
        ae->stamp = spikeEvent.first;
        ae->polarity = 0;
        ae->y = (spikeEvent.second / rowsize) << downsamplefactor;
        ae->x = (spikeEvent.second % rowsize) << downsamplefactor;
        outbottle.addEvent(ae);
    }
    std::cout << std::endl;

    //send the bottle on
    vBottleOut.write();


}

/**********************************************************/

void YARPspinO::threadRelease()
{
  // Clean up thread resources
  spinReceiver->closeRecvSocket();

}

/******************************************************************************/
// YARP - SPIN - INPUT/OUTPUT
/******************************************************************************/
YARPspinIO::YARPspinIO()
{

    //here we should initialise the module
    height = 128;
    width = 128;
    downsamplefactor = 0;

    spinSender = 0;
    spinReceiver = 0;
    //eventsin.open("eventssenttospinnaker.txt");

}

/******************************************************************************/
bool YARPspinIO::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    if(!yarp::os::BufferedPort<ev::vBottle>::open("/" + name + "/vBottle:i"))
        return false;
    if(!vBottleOut.open("/" + name + "/vBottle:o"))
        return false;

    return true;
}

/******************************************************************************/
void YARPspinIO::close()
{
    if(spinReceiver) spinReceiver->closeRecvSocket();
    if(spinSender) spinSender->closeSendSocket();
    delete spinSender; spinSender = 0;
    delete spinReceiver; spinReceiver = 0;
    vBottleOut.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    //remember to also deallocate any memory allocated by this class
}

/******************************************************************************/
void YARPspinIO::interrupt()
{
    vBottleOut.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

/**********************************************************/
void YARPspinIO::attachEIEIOmodules(spinnio::EIEIOSender* spinSenderPtr,
                                    spinnio::EIEIOReceiver *spinReceiverPtr)
{
    spinReceiver = spinReceiverPtr;
    spinSender = spinSenderPtr;
}

/**********************************************************/
void YARPspinIO::onRead(ev::vBottle &inbottle)
{
    //create event queue
    yarp::os::Stamp yts;
    this->getEnvelope(yts);

    vBottle &outbottle = vBottleOut.prepare();
    outbottle = inbottle;

    vQueue q = inbottle.get<AE>();
    if(!q.size()) {
        std::cerr << "Callback function received no packets?" << std::endl;
        return;
    }

    int latestts = q.back()->stamp;

    //first send on our packets we have read
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto v = is_event<AE>(*qi);
        if(v->getChannel()) continue;

        int neuronID = (v->y >> downsamplefactor) *
                (width / pow(downsamplefactor, 2.0)) +
                (v->x >> downsamplefactor);

        //eventsin << (int)(v->x) << " " << (int)(v->y) << " " << neuronID << std::endl;
        spinSender->addSpikeToSendQueue(neuronID);

    }

    //and then see if there are spikes to receive
    //int recvQueueSize = spinReceiver->getRecvQueueSize();

    //there is data to process so prepare out outbottle port
    auto gaborevent = make_event<GaussianAE>();

    if(spinReceiver->getRecvQueueSize()) {
        //convert the data to readable packets
        std::list<std::pair<int, int> > spikepacket =
                spinReceiver->getNextSpikePacket();

        //iterate over all spikes and add them to the bottle
        std::list<std::pair<int, int> >::iterator i;
        for(i = spikepacket.begin(); i != spikepacket.end(); i++) {
            std::pair<int,int> spikeEvent = *i;
            gaborevent->stamp = latestts;
            gaborevent->polarity = 0;
            gaborevent->x = 64;
            gaborevent->y = 64;
            if(spikeEvent.second == 0)  {
                gaborevent->sigxy = 0;
                gaborevent->sigy = 5;
                gaborevent->sigx = 5;
            } else {
                gaborevent->sigxy = 1;
                gaborevent->sigy = 5;
                gaborevent->sigx = 5;
            }
            outbottle.addEvent(gaborevent);
        }

    }

    //send the bottle on
    if(vBottleOut.getOutputCount())
        vBottleOut.write();

}


//empty line to make gcc happy
