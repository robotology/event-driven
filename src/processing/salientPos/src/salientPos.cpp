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

    bool iSuccess = inputManager.open(moduleName);


    return iSuccess;

}


/******************************************************************************/
bool vSalientPos::interruptModule()
{
    inputManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/******************************************************************************/
bool vSalientPos::close()
{
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

    prevTS = 0;
    thresh = 2000;
    decay = 1.0;
    clusterRadiusX = 10;
    clusterRadiusY = 10;
}

/******************************************************************************/
bool YARPsalientI::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    // open the output port here?
    std::string outPortName = "/" + name + "/vBottle:o";
    vBottleOut.open(outPortName);

    std::string inPortName = "/" + name + "/vBottle:i";
    return yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);
}

/******************************************************************************/
void YARPsalientI::close()
{

    yarp::os::BufferedPort<emorph::vBottle>::close();
    vBottleOut.close();

    //remember to also deallocate any memory allocated by this class
}

/******************************************************************************/
void YARPsalientI::interrupt()
{
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    vBottleOut.interrupt();

}


/**********************************************************/
void YARPsalientI::onRead(emorph::vBottle &bot)
{
    
    fullHeight = 128;
    fullWidth = 128;

    // create the x, y accumulator lists here

    xList = new std::vector<float>(fullWidth,0.0);
    yList = new std::vector<float>(fullHeight,0.0);

    //create event queue
    emorph::vQueue q = bot.getAll();

    int timeStamp = 0;
    
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        if(v->getChannel()) continue;

        int y = v->getY();
        int x = v->getX();

        // check what events we actually get

        //std::cout << "x " << x << " y " << y << " time " << v->getStamp() << std::endl;
        
        //timeStamp = v->getStamp();
        timeStamp += 10000;

        // accumulate spike events in arrays for x and y
        xList->at(x) += 1.0;
        yList->at(y) += 1.0;


       // decay according to temporal 'window'
       
       float timeDiff = float(v->getStamp() - prevTS);
       // if there's no time difference don't decay
       if (timeDiff > 0){

          decay = thresh/timeDiff;

       
          std::transform(xList->begin(), xList->end(), xList->begin(),std::bind1st    (std::multiplies<float>(), decay));

          std::transform(yList->begin(), yList->end(), yList->begin(),std::bind1st(std::multiplies<float>(), decay));
        }

        // calculate max x and y in terms of number of events

        std::vector<float>::iterator xbegin = xList->begin();
        std::vector<float>::iterator xend = xList->end();

        int prevMaxX = maxX;
    
        maxXCount = *(std::max_element(xbegin, xend));

        if (maxXCount > 2500.0){

            maxX = std::distance(xbegin,std::max_element(xbegin, xend));

        }

        // print out the current max x if something has changed

        if (maxX != prevMaxX){

          std::cout << "current max X " << maxX << " count "<< maxXCount << std::endl;
        }


        std::vector<float>::iterator ybegin = yList->begin();
        std::vector<float>::iterator yend = yList->end();
    

        int prevMaxY = maxY;

        maxYCount = *(std::max_element(ybegin, yend));

        if(maxYCount > 2500.0){

           maxY = std::distance(ybegin,std::max_element(ybegin, yend)); 

        } 

        // print out the current max y if something has changed

        if (maxY != prevMaxY){

            std::cout << "current max Y " << maxY << " count "<< maxYCount << std::endl; 
        }        

      prevTS = v->getStamp();

    } // event loop

    // Prepare the output port and create event

    if ((maxXCount > 2500.0) && (maxYCount > 2500.0)){
       emorph::vBottle &outbottle = vBottleOut.prepare();
       outbottle.clear();

       emorph::ClusterEventGauss cge;

       cge.setYCog(maxY);
       cge.setXCog(maxX);
       cge.setStamp(timeStamp);
       cge.setChannel(0);
       cge.setXSigma2(clusterRadiusX);
       cge.setYSigma2(clusterRadiusY);
       outbottle.addEvent(cge);
    }

    //send the bottle on
    vBottleOut.write();
   
}

//empty line to make gcc happy
