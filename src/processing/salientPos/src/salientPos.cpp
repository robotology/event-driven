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


    fullHeight = 128;
    fullWidth = 128;

    // create the x, y accumulator lists here

    xList = new std::vector<int>(fullWidth,0);
    yList = new std::vector<int>(fullHeight,0);


    // pass in x, y arrays to inputManager
    inputManager.attachXYLists(xList, yList, fullWidth, fullHeight);
    bool iSuccess = inputManager.open(moduleName);

    // pass the x, y accumulator arrays to initThread
    bool oSuccess = outputManager.initThread(moduleName, xList, yList, fullWidth, fullHeight);

    // Start the output manager thread
    outputManager.start();
    outputManager.run();


    //bool oSuccess = true;
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

        // check what events we actually get

        std::cout << "x " << x << " y " << y << std::endl;

        // accumulate spike events in arrays for x and y
        xList->at(x)++;
        yList->at(y)++;

    }
   
}

/**********************************************************/
void YARPsalientI::attachXYLists(std::vector<int>* xListPtr, std::vector<int>* yListPtr, int xSizeIn, int ySizeIn)
{
   xList = xListPtr;
   yList = yListPtr;
   xSize = xSizeIn;
   ySize = ySizeIn;


}

/**********************************************************/
YARPsalientO::YARPsalientO() : yarp::os::RateThread(1)
{

  // Do we need anything here?
    
}

/**********************************************************/

bool YARPsalientO::initThread(std::string moduleName, std::vector<int>* xListPtr, std::vector<int>* yListPtr, int xSizeIn, int ySizeIn)
{
    // Output thread initialisation

    xList = xListPtr;
    yList = yListPtr;
    xSize = xSizeIn;
    ySize = ySizeIn;

    std::string outPortName = "/" + moduleName + "/vBottle:o";
    return vBottleOut.open(outPortName);
}


/**********************************************************/

void YARPsalientO::run()
{
    std::vector<int>::iterator xbegin = xList->begin();
    std::vector<int>::iterator xend = xList->end();
    

    maxXCount = *(std::max_element(xbegin, xend));

    maxX = std::distance(xbegin,std::max_element(xbegin, xend));


    std::vector<int>::iterator ybegin = yList->begin();
    std::vector<int>::iterator yend = yList->end();
    

    maxYCount = *(std::max_element(ybegin, yend));

    maxY = std::distance(ybegin,std::max_element(ybegin, yend)); 

    // get the current max x and y

    std::cout << "current max X " << maxX << " count "<< maxXCount << std::endl;

    std::cout << "current max Y " << maxY << " count "<< maxYCount << std::endl;


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
