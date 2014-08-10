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

#include "eventBottleConverter.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace emorph::ecodec;

/**********************************************************/
bool EventBottleConverter::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("eventBottleConverter"), "module name (string)").asString();

    setName(moduleName.c_str());

    rpcPortName  =  "/";
    rpcPortName +=  getName();
    rpcPortName +=  "/rpc:i";

    if (!rpcPort.open(rpcPortName.c_str()))
    {
        fprintf(stdout, "%s : Unable to open port %s\n", getName().c_str(), rpcPortName.c_str());
        return false;
    }

    attach(rpcPort);

    /* create the thread and pass pointers to the module parameters */
    etbHandler = new EventToBottleHandler( moduleName );
    bteHandler = new BottleToEventHandler( moduleName );

    /* now open the manager to do the work */
    etbHandler->open();
    bteHandler->open();

    return true ;
}

/**********************************************************/
bool EventBottleConverter::interruptModule()
{
    rpcPort.interrupt();
    etbHandler->interrupt();
    bteHandler->interrupt();
    return true;
}

/**********************************************************/
bool EventBottleConverter::close()
{
    rpcPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");

    etbHandler->close();
    bteHandler->close();

    fprintf(stdout, "deleting thread\n");
    delete etbHandler;
    delete bteHandler;
    fprintf(stdout, "done deleting thread\n");
    return true;
}

/**********************************************************/
bool EventBottleConverter::updateModule()
{
    return !closing;
}

/**********************************************************/
double EventBottleConverter::getPeriod()
{
    return 0.1;
}

/**********************************************************/

EventToBottleHandler::~EventToBottleHandler()
{

}

/**********************************************************/
EventToBottleHandler::EventToBottleHandler( const string &moduleName )
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
}

/**********************************************************/
bool EventToBottleHandler::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + "/etb:i";
    BufferedPort<eventBottle >::open( inPortName.c_str() );

    outPortName = "/" + moduleName + "/etb:o";
    outPort.open( outPortName.c_str() );

    return true;
}

/**********************************************************/
void EventToBottleHandler::close()
{
    fprintf(stdout,"now closing ports...\n");
    outPort.close();
    BufferedPort<eventBottle >::close();
    fprintf(stdout,"finished closing the read port...\n");
}

/**********************************************************/
void EventToBottleHandler::interrupt()
{
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    BufferedPort<eventBottle >::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
}

/**********************************************************/
void EventToBottleHandler::onRead(eventBottle &bot)
{

    double currTime = yarp::os::Time::now();
    //create event queue
    eEventQueue q; 
    unsigned long ts;
    //decode packet
    Bottle out; 
    Stamp st;
    BufferedPort<eventBottle >::getEnvelope(st);

    double timeaex = st.getTime(); 
    //fprintf(stdout, "aexGrab %lf \n",timeaex);
    //fprintf(stdout, "%d\n", bot.get_sizeOfPacket()) ;
    if(eEvent::decode(*bot.get_packet(),q)) 
    {   
        int size = q.size();
        //fprintf(stdout, "eventBottle size: %d\n",size);
        Bottle &pack =  out.addList();  
        for (int e = 0; e < size; e++)
        {
            if(q[e] != 0)
            {
                if(q[e]->getType()=="TS") //identify the type of the packet (Time Stamp)
                {
                    TimeStamp* ptr=dynamic_cast<TimeStamp*>(q[e]);
                    ts = (unsigned long) ptr->getStamp();
                    //fprintf(stdout, "Size %d TimeStamp: %d", size, (unsigned int) ptr->getStamp());
                }
                else if (q[e]->getType()=="AE") //identify the type of the packet (Address Event)
                {
                    AddressEvent* ptr=dynamic_cast<AddressEvent*>(q[e]); //create the Address Event for the type
                    if(ptr->isValid()) 
                    {
                        int posX = ptr->getX();
                        int posY = ptr->getY();
                        int polarity = ptr->getPolarity();
                        int channel = ptr->getChannel();
                        string type = ptr->getType();
                        
                        Bottle &tmp = pack.addList();
                        
                        Bottle &bts = tmp.addList();  
                        bts.addString("time");                      
                        bts.addDouble(ts);
                        
                        Bottle &bposX = tmp.addList(); 
                        bposX.addString("posX");
                        bposX.addInt(posX);
                        
                        Bottle &bposY = tmp.addList();
                        bposY.addString("posY");
                        bposY.addInt(posY);
                        
                        Bottle &bpol = tmp.addList();
                        bpol.addString("polarity");
                        bpol.addInt(polarity);
                        
                        Bottle &bch = tmp.addList();
                        bch.addString("channel");
                        bch.addInt(channel);
                        
                        Bottle &btype = tmp.addList();
                        btype.addString("type");
                        btype.addString(type);
                
                        //fprintf(stdout, "Size %d TimeStamp: %d AddressEvent: (X: %d  Y: %d  pol: %d  cha: %d) \n",size, ts, posX, posY, polarity, channel);
                    } 
                }
                else if (q[e]->getType()=="CLE-G") //identify the type of the packet (ClusterEventGauss)
                {   
                    ClusterEventGauss* ptr=dynamic_cast<ClusterEventGauss*>(q[e]); //create the Event for the type
                    if(ptr->isValid()) 
                    {
                        int channel = ptr->getChannel();        
                        //int getId()       id;             
                        int xCog = ptr->getXCog();           
                        int yCog = ptr->getYCog();           
                        string type = ptr->getType();
                        int numAE = ptr->getNumAE();            
                        int xSigma2 = ptr->getXSigma2(); 
                        int ySigma2 = ptr->getYSigma2();          
                        int xySigma = ptr->getXYSigma();         
                                            
                        Bottle &tmp = pack.addList();
                        
                        Bottle &bts = tmp.addList();  
                        bts.addString("time");                      
                        bts.addDouble(ts);
                        
                        Bottle &bxCog = tmp.addList(); 
                        bxCog.addString("xCog");
                        bxCog.addInt(xCog);
                        
                        Bottle &byCog = tmp.addList();
                        byCog.addString("yCog");
                        byCog.addInt(yCog);
                        
                        Bottle &bnumAE = tmp.addList();
                        bnumAE.addString("numAE");
                        bnumAE.addInt(numAE);

                        Bottle &bxSigma2 = tmp.addList();
                        bxSigma2.addString("xSigma2");
                        bxSigma2.addInt(xSigma2);

                        Bottle &bySigma2 = tmp.addList();
                        bySigma2.addString("ySigma2");
                        bySigma2.addInt(ySigma2);

                        Bottle &bxySigma = tmp.addList();
                        bxySigma.addString("xySigma");
                        bxySigma.addInt(xySigma);
                        
                        Bottle &bch = tmp.addList();
                        bch.addString("channel");
                        bch.addInt(channel);
                        
                        Bottle &btype = tmp.addList();
                        btype.addString("type");
                        btype.addString(type);
                
                        //fprintf(stdout, "Size %d TimeStamp: %d ClusterEventGauss: (X: %d  Y: %d  cha: %d m: %d xs: %d ys: %d xys: %d) \n",size, ts, xCog, yCog, channel, numAE, xSigma2, ySigma2, xySigma);
                    } 
                }

            }
        } 
        double timeaex = st.getTime(); 
        //fprintf(stdout, "aexsend %lf \n",timeaex);
        outPort.setEnvelope(st);
        outPort.write(out);
    }   

    double ms = (yarp::os::Time::now() - currTime)*1000;
    //fprintf(stdout, "------------------------------------------------------------------\n");*/
    //fprintf(stdout, "time spent is %lf \n",ms);
}
BottleToEventHandler::~BottleToEventHandler()
{

}


/**********************************************************/
BottleToEventHandler::BottleToEventHandler( const string &moduleName )
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
}

/**********************************************************/
bool BottleToEventHandler::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + "/bte:i";
    BufferedPort<Bottle >::open( inPortName.c_str() );

    outPortName = "/" + moduleName + "/bte:o";
    outPort.open( outPortName.c_str() );

    return true;
}

/**********************************************************/
void BottleToEventHandler::close()
{
    mutex.wait();
    fprintf(stdout,"now closing ports...\n");
    outPort.close();
    BufferedPort<Bottle >::close();
    fprintf(stdout,"finished closing the read port...\n");
    mutex.post();
}

/**********************************************************/
void BottleToEventHandler::interrupt()
{
    mutex.wait();
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    BufferedPort<Bottle >::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
    mutex.post();
}

/**********************************************************/
void BottleToEventHandler::onRead(Bottle &bot)
{   
    mutex.wait();
    
    eventBottle &out = outPort.prepare();
    
    Bottle *main = bot.get(0).asList(); //read main list from bottle
    int size = main->size();

    Bottle event;
    
    for (int b=0; b<size;b++)
    {
        Bottle *cur = main->get(b).asList(); //get each sublist  
        //fprintf(stdout, "Bottle: %s\n", cur->toString().c_str());
        
        //create timestamp       
        TimeStamp ts;
        ts.setStamp(cur->find("time").asDouble());

        string type = cur->find("type").asString().c_str(); //for each sublist get type  
        //create Bottle to append Timestamp and Event
        Bottle tmp;
        //encode TimeStamp 
        tmp = ts.encode();
        if (type=="AE")
        {
            //create addressEvent for specific type
            AddressEvent evt; 
            //start filling addressEvent with data from sublists
            evt.setX(cur->find("posX").asInt());
            evt.setY(cur->find("posY").asInt());
            evt.setPolarity(cur->find("polarity").asInt());
            evt.setChannel(cur->find("channel").asInt());
            //append timeStamp and Event into single bottle
            tmp.append(evt.encode());
        }

        if (type=="CLE-G")
        {
            //create addressEvent for specific type
            ClusterEventGauss evt; 
            //start filling addressEvent with data from sublists
            evt.setChannel(cur->find("channel").asInt());
            evt.setXCog(cur->find("xCog").asInt());
            evt.setYCog(cur->find("yCog").asInt());
            evt.setNumAE(cur->find("numAE").asInt());
            evt.setXSigma2(cur->find("xSigma2").asInt());
            evt.setYSigma2(cur->find("ySigma2").asInt());
            evt.setXYSigma(cur->find("xySigma").asInt());
            
            //append timeStamp and Event into single bottle
            tmp.append(evt.encode());
        }


        //append the bottle tmp with TimeStamp and Event of a single spike to a bottle that contains the events of the whole packet 
        event.append(tmp);
    }
    //create and fill in dataTmp (eventBottle) with data from event (Bottle) 
    //fprintf(stdout, "\n\n\nEvent: %s\n", event.toString().c_str());
    eventBottle dataTmp(&event);
    //copy dataTmp to eventBottle out
    out = dataTmp;
    //send it all out
    outPort.write();
    mutex.post();
}

//empty line to make gcc happy
