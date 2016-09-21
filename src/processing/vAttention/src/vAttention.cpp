/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
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

#include "vAttention.h"

using namespace yarp::math;

/**********************************************************/
bool vAttentionModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vAttention")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();
    
    /* name of the file where the feature map is stored */
    std::string mapName =
            rf.check("mode", yarp::os::Value("intensity")).asString();
    mapName += ".txt";
    std::string mapNameComplete = rf.findFile(mapName.c_str());
    
    /* attach a port of the same name as the module (prefixed with a /) to the module 
     so that messages received from the port are redirected to the respond method */
    
    std::string handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal
    
    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }
    
    attach(handlerPort);                  // attach to port

    /* set parameters */
    int sensorSize = rf.check("sensorSize", yarp::os::Value(128)).asInt();
    int filterSize = rf.check("filterSize", yarp::os::Value(3)).asInt();
    double tau = rf.check("tau", yarp::os::Value(3)).asDouble();
    double thrSal = rf.check("thr", yarp::os::Value(0.3)).asDouble();
    
    /* create the thread and pass pointers to the module parameters */
    attManager = new vAttentionManager(sensorSize, filterSize, tau, thrSal);
    
    return attManager->open(moduleName, strict);

}

/**********************************************************/
bool vAttentionModule::interruptModule()
{
    attManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vAttentionModule::close()
{
    attManager->close();
    delete attManager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vAttentionModule::updateModule()
{
    return true;
}

/**********************************************************/
double vAttentionModule::getPeriod()
{
    return 1;
}
/**********************************************************/
bool vAttentionModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) {
    std::string helpMessage =  std::string(getName().c_str()) +
    " commands are: \n" +
    "help \n" +
    "quit \n";
    
    reply.clear();
    
    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }
    return true;
}

/******************************************************************************/
//vAttentionManager
/******************************************************************************/
vAttentionManager::vAttentionManager(int sensorSize, int filterSize, double tau, double thrSal)
{
    this -> sensorSize = sensorSize;
    this -> filterSize = filterSize;
    this -> tau = tau;
    this -> thrSal = thrSal;
    
    subMatrixSize = filterSize/2;
    normSal = thrSal/255;
    
    ptime = 0; // past time stamp
    
    
    //for speed we predefine the mememory for some matricies
    salMapLeft = yarp::sig::Matrix(sensorSize, sensorSize); // from vFlow
    salMapRight = yarp::sig::Matrix(sensorSize, sensorSize); // from vFlow
    filterMap = yarp::sig::Matrix(filterSize, filterSize); // from vFlow
    
    // initialise saliency map to zero
    salMapLeft.zero();
    salMapRight.zero();
    

    // compute the filter
    for(int r = 0; r < filterSize; r++) {
        for(int c = 0; c < filterSize; c++) {
            filterMap(r,c) = 0.1;
        }
    }
}
/**********************************************************/
bool vAttentionManager::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    // why is the input port treated differently???? both in open and close
    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    outPortName = "/" + moduleName + "/salMapLeft:o";
    bool check3 = outSalMapLeftPort.open(outPortName);

    outPortName = "/" + moduleName + "/salMapRight:o";
    bool check4 = outSalMapRightPort.open(outPortName);
    
    std::cout << "opened ports: " << std::endl << "vBottle:i " << check1  << std::endl << "vBottle:o "<< check2  << std::endl<< "/salMapLeft:o " << check3  << std::endl<< "/salMapRight:o " << check4 << std::endl;
    
    //salMapImageLeft   = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    //salMapImageRight  = new yarp::sig::ImageOf<yarp::sig::PixelMono>;

    //salMapImageLeft ->resize(sensorSize,sensorSize);
    //salMapImageRight->resize(sensorSize,sensorSize);
    
    // ---- initialise the images of the saliency maps left and right to 0 ---- //
    //memset((void*)salMapImageLeft, 0, sensorSize * sensorSize * sizeof(unsigned char));
    //memset((void*)salMapImageRight, 0, sensorSize * sensorSize * sizeof(unsigned char));
    
    std::cout << "initialisation correctly ended" << std::endl;
    
    return check1 && check2 && check3 && check4;
}

/**********************************************************/
void vAttentionManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
    outSalMapLeftPort.close();
    outSalMapRightPort.close();
    
    //free(salMapImageLeft);
    //free(salMapImageRight);
}

/**********************************************************/
void vAttentionManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    outSalMapLeftPort.interrupt();
    outSalMapRightPort.interrupt();
}

/**********************************************************/
void vAttentionManager::onRead(emorph::vBottle &bot)
{
    /*prepare output vBottle with AEs */
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);
    
    /* get the event queue in the vBottle bot */
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.sort(true);
    //q.back()->getStamp();
    
    //unsigned long int dt  = unwrap(q.back()->getStamp()) - ptime;
    //ptime = unwrap.currentTime();
    
    unsigned long int t  = unwrap(q.back()->getStamp());
    unsigned long int dt  = t - ptime;
    ptime = t;
    
    // ---- decay saliency map ---- //
    
    decaySaliencyMap(salMapLeft,dt);
    decaySaliencyMap(salMapRight,dt);
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        // --- increase energy of saliency map  --- //
        
        if (aep->getChannel() == 0){
            updateSaliencyMap(salMapLeft,aep);
        }
        else{
            updateSaliencyMap(salMapRight,aep);
        }
    }
    std::cout << "update done" << std::endl;
    
    // ---- normalise saliency map ---- //
    
    //normaliseSaliencyMap(salMapLeft);
    
    // ---- adding the event to the output vBottle if it passes thresholds ---- //
    
    /*
     if(pFeaOn[posFeaImage] > thrOn) {
     std::cout << "adding On event to vBottle" << std::endl;
     
     emorph::AddressEvent ae = *aep;
     ae.setPolarity(1);
     ae.setX(xevent);
     ae.setY(yevent);
     
     outBottle->addEvent(ae);
     
     }
     else if(pFeaOff[posFeaImage] < thrOff) {
     std::cout << "adding Off event to vBottle" << std::endl;
     emorph::AddressEvent ae = *aep;
     ae.setPolarity(0);
     ae.setX(xevent);
     ae.setY(yevent);
     
     outBottle->addEvent(ae);
     
     }
     */
    
    //  --- convert to images for display --- //
    /*
    unsigned char* pSalImgLeft   = salMapImageLeft->getRawImage();
    unsigned char* pSalImgRight  = salMapImageRight->getRawImage();

    for(int r = 0; r < sensorSize; r++) {
        for(int c = 0; c < sensorSize; c++) {
            *pSalImgLeft = std::min(salMapLeft(r,c),thrSal)/normSal;
            pSalImgLeft++;
            *pSalImgRight = std::min(salMapRight(r,c),thrSal)/normSal;
            pSalImgRight++;
        }
    }
    */
    // --- writing vBottle on buffered output port
    if (strictness) outPort.writeStrict();
    else outPort.write();
    
    // --- writing images of left and right saliency maps on output port
    if(outSalMapLeftPort.getOutputCount()) {
        std::cout << "sending left image"<< std::endl;
        outSalMapLeftPort.prepare()  = *salMapImageLeft;
        outSalMapLeftPort.write();
    }
    if(outSalMapRightPort.getOutputCount()) {
        std::cout << "sending right image"<< std::endl;
        outSalMapRightPort.prepare()  = *salMapImageRight;
        outSalMapRightPort.write();
    }
}

/**********************************************************/
void vAttentionManager::updateSaliencyMap(yarp::sig::Matrix &salMap, emorph::AddressEvent *aep)
{
    // unmask event: get x, y, pol, channel
    int cartY     = aep->getX();
    int cartX     = aep->getY();
    
    int rf, cf;
    rf = 0;
    cf = 0;
    
    int r1, r2, c1, c2;
    r1 = cartX-subMatrixSize;
    r2 = cartX+subMatrixSize;
    c1 = cartY-subMatrixSize;
    c2 = cartY-subMatrixSize;
    
    // ---- increase energy in the location of the event ---- //
    
    for(int r = r1; r < filterSize; r++) {
        for(int c = c1; c < filterSize; c++) {
            salMap(r,c) += filterMap(rf,cf);
            rf ++;
            cf ++;
        }
    }
    
}

void vAttentionManager::decaySaliencyMap(yarp::sig::Matrix &salMap, unsigned long int dt)
{
    //    std::cout << "saliency map decay" << std::endl;
    double expDecay = exp(dt/tau);
    salMap = salMap * expDecay;
    
}

/*
void vAttentionManager::normaliseSaliencyMap(yarp::sig::Matrix &salMap)
{
    double min;
    double max;
    
    min = salMap(0,0);
    max = min;
    
    // ---- find max and min values of saliency map ---- //
    for(int r = 0; r < sensorSize; r++) {
        for(int c = 0; c < sensorSize; c++) {
            if (salMap(r,c) > max){
                max = salMap(r,c);
            }
            if (salMap(r,c) < min){
                min = salMap(r,c);
            }
        }
    }
    
    // ---- normalise ---- //
    for(int r = 0; r < sensorSize; r++) {
        for(int c = 0; c < sensorSize; c++) {
            salMap(r,c) = salMap(r,c) - min / (max - min) ;
        }
    }
}
*/

//empty line to make gcc happy
