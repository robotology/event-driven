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

#include "salientPos.h"

/******************************************************************************/
// vSpinInterface
/******************************************************************************/
bool vSalientPos::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("salientPos")).asString();
    setName(moduleName.c_str());

    //downsampleHeight = 32;
    //downsampleWidth = 32;

    fullHeight = 128;
    fullWidth = 128;

    // create the x, y accumulator lists here

    xList = new std::vector<int>(fullWidth,0);
    yList = new std::vector<int>(fullHeight,0);

    // pass the x, y accumulator arrays to initThread
    bool oSuccess = outputManager.initThread(moduleName, xList, yList);

    // Start the output manager thread
    outputManager.start();
    outputManager.run();

    // pass in x, y arrays to inputManager
    inputManager.attachXYLists(xList, yList);
    bool iSuccess = inputManager.open(moduleName);

    return oSuccess && iSuccess;

}


/******************************************************************************/
bool vSalientPos::interruptModule()
{
    outputManager.stop();
    inputManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/******************************************************************************/
bool vSalientPos::close()
{
    outputManager.threadRelease();
    inputManager.close();
    yarp::os::RFModule::close();
    return true;
}

/******************************************************************************/
bool vSalientPos::updateModule()
{
    return true;
}

/******************************************************************************/
double vSalientPos::getPeriod()
{
    return 1;
}

/******************************************************************************/
// YARP - SALIENT - INPUT
/******************************************************************************/
YARPsalientI::YARPsalientI()
{

    //here we should initialise the module


    
}

/******************************************************************************/
bool YARPsalientI::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    return yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);
}

/******************************************************************************/
void YARPsalientI::close()
{

    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class
}

/******************************************************************************/
void YARPsalientI::interrupt()
{
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}


/**********************************************************/
void YARPsalientI::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q = bot.getAll();
    

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        if(v->getChannel()) continue;

        int y = v->getY();
        int x = v->getX();

        // accumulate spike events in arrays for x and y?

        xList->at(x)++;
        yList->at(y)++;



    }

}

/**********************************************************/
void YARPsalientI::attachXYLists(std::vector<int>* xListPtr, std::vector<int>* yListPtr)
{
   xList = xListPtr;
   yList = yListPtr;
}

/**********************************************************/
YARPsalientO::YARPsalientO() : yarp::os::RateThread(1)
{

  // Do we need anything here?
    
}

/**********************************************************/

bool YARPsalientO::initThread(std::string moduleName, std::vector<int>* xListPtr, std::vector<int>* yListPtr)
{
    // Output thread initialisation

    xList = xListPtr;
    yList = yListPtr;

    std::string outPortName = "/" + moduleName + "/vBottle:o";
    return vBottleOut.open(outPortName);
}


/**********************************************************/

void YARPsalientO::run()
{

    // Check we can access the xList and yList

    //std::cout << "x list " << xList->size() << std::endl;

    /*
    int maxXCount = 0;
    int maxX = 0;
    for(std::vector<int>::iterator i = xList->begin(); i != xList->end(); i++) {

        int count = *i;
        if(count > maxXCount){
           maxXCount = count;
           maxX = 
        }
        //std::cout << "x item " << count << std::endl;        

    }
    */

    std::vector<int>::iterator xbegin = xList->begin();
    std::vector<int>::iterator xend = xList->end();
    

    //int maxX = *(std::max_element(xbegin, xend));

    int maxX = std::distance(std::max_element(xbegin, xend), xend);

    //std::cout << "y list " << yList->size() << std::endl;
    /*
    int maxY = 0;
    for(std::vector<int>::iterator i = yList->begin(); i != yList->end(); i++) {

        int count = *i;
        if(count > maxX){
           maxX = count;
        }
        //std::cout << "y item " << count << std::endl;        

    }
    */

    std::vector<int>::iterator ybegin = yList->begin();
    std::vector<int>::iterator yend = yList->end();
    

    //int maxY = *(std::max_element(ybegin, yend));

    int maxY = std::distance(std::max_element(ybegin, yend), yend); 

    // get the current max x and y

    std::cout << "current max X " << maxX-1 << " count "<< xList->at(maxX-1) << std::endl;

    std::cout << "current max Y " << maxY-1 << " count "<< yList->at(maxY-1) << std::endl;


    // Prepare the output port and create event

    emorph::vBottle &outbottle = vBottleOut.prepare();
    outbottle.clear();

    emorph::ClusterEventGauss cge;

    cge.setYCog(maxY);
    cge.setXCog(maxX);
    outbottle.addEvent(cge);

    //send the bottle on
    vBottleOut.write();

}

/**********************************************************/

void YARPsalientO::threadRelease()
{
  // Clean up thread resources
 
}


//empty line to make gcc happy
