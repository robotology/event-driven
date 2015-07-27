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

/**********************************************************/
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


    initSpin(17895, 12346, "192.168.1.1",
                       "/home/ubuntu/VVV/TestNetwork/application_generated_data_files/latest/input_output_database.db",
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


    //DO WE NEED THIS CODE? IF SO IT SHOULD BE IN INITSPIN
    int sendQueueSize = spinSender->getSendQueueSize();
    cout << "Send queue size is " << sendQueueSize << endl;



    // Start the output manager thread
    outputManager.start();
    bool oSuccess = outputManager.initThread(moduleName, spinReceiver);
    outputManager.run();

    inputManager.attachEIEIOSender(spinSender);
    bool iSuccess = inputManager.open(moduleName);

    return oSuccess && iSuccess;

}

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

/**********************************************************/
bool vSpinInterface::interruptModule()
{
    // interrupt the input port
    inputManager.interrupt();
    // what do we need to do with output thread?
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vSpinInterface::close()
{
    outputManager.threadRelease();
    inputManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vSpinInterface::updateModule()
{
    return true;
}

/**********************************************************/
double vSpinInterface::getPeriod()
{
    return 1;
}

/**********************************************************/
YARPspinI::YARPspinI()
{

    //here we should initialise the module
    height = 128;
    width = 128;
    downsamplefactor = 2;
    eventsin.open("eventssenttospinnaker.txt");
    
}

/**********************************************************/
bool YARPspinI::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool state1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    //std::string outPortName = "/" + name + "/vBottle:o";
    //bool state2 = outPort.open(outPortName);
    bool state2 = true;
    return state1 && state2;
}

/**********************************************************/
void YARPspinI::close()
{
    //close ports
    eventsin.close();
    spinSender->closeSendSocket();
    //outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class
}

/**********************************************************/
void YARPspinI::interrupt()
{
    //pass on the interrupt call to everything needed
    //outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void YARPspinI::attachEIEIOSender(spinnio::EIEIOSender* spinSenderPtr)
{
   spinSender = spinSenderPtr;

}

/**********************************************************/
void YARPspinI::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q = bot.getAll();
    

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        if(v->getChannel()) continue;

        int neuronID = (v->getY() >> downsamplefactor) *
                (width / pow(downsamplefactor, 2.0)) +
                (v->getX() >> downsamplefactor);

        //eventsin << (int)(v->getX()) << " " << (int)(v->getY()) << " " << neuronID << std::endl;
        spinSender->addSpikeToSendQueue(neuronID);

    }

}
/**********************************************************/
YARPspinO::YARPspinO() : yarp::os::RateThread(1)
{

  // Do we need anything here?
    
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

   // Main thread code - polling EIEIOReceiver queue?
   int recvQueueSize = spinReceiver->getRecvQueueSize();
   if (recvQueueSize > 0)
   {
      cout << "Polling Spin receiver queue, size is " << recvQueueSize << endl ;
   }

   //add to a vBottle Here!


}

/**********************************************************/

void YARPspinO::threadRelease()
{
  // Clean up thread resources
  spinReceiver->closeRecvSocket();

}


//empty line to make gcc happy
