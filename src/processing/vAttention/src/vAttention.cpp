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
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

using namespace yarp::math;
using namespace std;

/**********************************************************/
bool vAttentionModule::configure(yarp::os::ResourceFinder &rf) {
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vAttention")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
                  rf.check("strict", yarp::os::Value(true)).asBool();

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
    int filterSize = rf.check("filterSize", yarp::os::Value(9)).asInt();
    double tau = rf.check("tau", yarp::os::Value(1000000)).asDouble();
    double thrSal = rf.check("thr", yarp::os::Value(20)).asDouble();

    /* create the thread and pass pointers to the module parameters */
    attManager = new vAttentionManager(sensorSize, filterSize, tau, thrSal);

    return attManager->open(moduleName, strict);

}

/**********************************************************/
bool vAttentionModule::interruptModule() {
    attManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vAttentionModule::close() {
    attManager->close();
    delete attManager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vAttentionModule::updateModule() {
    return true;
}

/**********************************************************/
double vAttentionModule::getPeriod() {
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
vAttentionManager::vAttentionManager(int sensorSize, int filterSize, double tau, double thrSal) {
    this -> sensorSize = sensorSize;
    this -> filterSize = filterSize;
    this -> tau = tau;
    this -> thrSal = thrSal;
    this -> shift = filterSize/2;

    normSal = thrSal/255;

    ptime = 0; // past time stamp


    //for speed we predefine the memory for some matrices
    salMapLeft  = yarp::sig::Matrix(sensorSize+filterSize, sensorSize+filterSize);
    salMapRight = yarp::sig::Matrix(sensorSize+filterSize, sensorSize+filterSize);
    filterMap   = yarp::sig::Matrix(filterSize, filterSize);

    // initialise saliency map to zero
    salMapLeft.zero();
    salMapRight.zero();
//    loadFilter();

   // double sigma = 2;
    
    for (int r = 0; r < filterSize; r++){
        for (int c = 0; c < filterSize; c++){
//            double center = filterSize/2;
//            double rDist = r - center;
//            double cDist = c - center;
//            filterMap (r,c) = exp(-(pow(rDist,2)/(2*pow(sigma,2)) + pow(cDist,2)/(2*pow(sigma,2))));
            filterMap(r,c) = 0.1;
        }
    }

    printSaliencyMap(filterMap);

}

bool vAttentionManager::loadFilter()  {

    //TODO
    const int MAX_CHARS_PER_LINE = 512;
//    const int MAX_TOKENS_PER_LINE = 20;
    const char* const DELIMITER = " ";

    // create a file-reading object
    ifstream fin;
    fin.open("data.txt"); // open a file
    if (!fin.good())
        return false; // exit if file not found

    double numTokens;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);

        // parse the line into blank-delimited tokens
        int n = 0; // a for-loop index

        // array to store memory addresses of the tokens in buf
        char* token = 0;

        // parse the line
        token = strtok(buf, DELIMITER); // first token
        numTokens = 0;
        while(token){
            token = strtok(buf,DELIMITER);
            numTokens ++;
            this ->filterSize = numTokens;
        }


        // process (print) the tokens
        for (int i = 0; i < n; i++) // n = #of tokens
            cout << "Token[" << i << "] = " << token[i] << endl;
        cout << endl;
    }
    return true;
}

/**********************************************************/
bool vAttentionManager::open(const std::string moduleName, bool strictness) {
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

    salMapImageLeft   = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    salMapImageRight  = new yarp::sig::ImageOf<yarp::sig::PixelMono>;

    salMapImageLeft ->resize(sensorSize,sensorSize);
    salMapImageRight->resize(sensorSize,sensorSize);

    salMapImageLeft ->zero();
    salMapImageRight->zero();

    // ---- initialise the images of the saliency maps left and right to 0 ---- //
    //memset((void*)salMapImageLeft, 0, sensorSize * sensorSize * sizeof(unsigned char));
    //memset((void*)salMapImageRight, 0, sensorSize * sensorSize * sizeof(unsigned char));

    std::cout << "initialisation correctly ended" << std::endl;

    return check1 && check2 && check3 && check4;
}

/**********************************************************/
void vAttentionManager::close() {
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
    outSalMapLeftPort.close();
    outSalMapRightPort.close();

    //free(salMapImageLeft);
    //free(salMapImageRight);
}

/**********************************************************/
void vAttentionManager::interrupt() {
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    outSalMapLeftPort.interrupt();
    outSalMapRightPort.interrupt();
}

/**********************************************************/
void vAttentionManager::onRead(emorph::vBottle &bot) {
    /* get the event queue in the vBottle bot */
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.sort(true);

    unsigned long int t  = unwrap(q.back()->getStamp());
    unsigned long int dt  = t - ptime;
    ptime = t;

    int numEvent = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        // --- increase energy of saliency map  --- //
        if (aep->getChannel() == 0) {
            updateSaliencyMap(salMapLeft,aep);
        }
        else {
            updateSaliencyMap(salMapRight,aep);
        }
        numEvent ++;
    }

    //std::cout << "numEvent = " << numEvent << std::endl;


    decaySaliencyMap(salMapLeft, dt);
    decaySaliencyMap(salMapRight, dt);


    // ---- normalise saliency map ---- //

//    normaliseSaliencyMap(salMapLeft);
//    normaliseSaliencyMap(salMapRight);

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

    displaySaliencymap();

}

void vAttentionManager::displaySaliencymap() {

    /*prepare output vBottle with AEs */
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st);
    outPort.setEnvelope(st);

    unsigned char* pSalImgLeft   = salMapImageLeft->getRawImage();
    unsigned char* pSalImgRight  = salMapImageRight->getRawImage();

    for(int r = shift; r < sensorSize + shift; r++) {
        for(int c = shift; c < sensorSize + shift; c++) {
            *pSalImgLeft = std::min(salMapLeft(r, c), thrSal)/normSal;
//            std::cout << "salMapLeft ("<<r<<","<<c<<"): " << *pSalImgLeft << " - " << pSalImgLeft << " - " << &pSalImgLeft << " - " << std::min(salMapLeft(r,c),thrSal)/*/normSal*/ << std::endl;
//            std::cout << std::min (salMapLeft(r,c), thrSal) << " ";

            pSalImgLeft++;
            *pSalImgRight = std::min(salMapRight(r, c), thrSal)/normSal;
            pSalImgRight++;
        }
//        std::cout << std::endl;
    }

    // --- writing vBottle on buffered output port
    if (strictness) {
        outPort.writeStrict();
    } else {
        outPort.write();
    }

    // --- writing images of left and right saliency maps on output port
    if(outSalMapLeftPort.getOutputCount()) {
        //std::cout << "sending left image"<< std::endl;
        outSalMapLeftPort.prepare()  = *salMapImageLeft;
        outSalMapLeftPort.write();
    }

    if(outSalMapRightPort.getOutputCount()) {
        //std::cout << "sending right image"<< std::endl;
        outSalMapRightPort.prepare()  = *salMapImageRight;
        outSalMapRightPort.write();
    }
}

/**********************************************************/
void vAttentionManager::updateSaliencyMap(yarp::sig::Matrix &salMap, emorph::AddressEvent *aep) {
    // unmask event: get x, y, pol, channel
    int cartX = aep->getX() + shift;
    int cartY = aep->getY() + shift;

    int rf, cf;
    rf = 0;

    // ---- increase energy in the location of the event ---- //

    for(int r = cartX - shift; r < cartX + filterSize - shift; r++) {
        cf = 0;
        for(int c = cartY - shift; c < cartY + filterSize - shift; c++) {
            salMap(r,c) += filterMap(rf,cf);
            cf ++;
        }
        rf ++;
    }
}

void vAttentionManager::printSaliencyMap (yarp::sig::Matrix &salMap) {
    for (int r = 0; r < salMap.rows(); r ++){
        for (int c = 0; c < salMap.cols(); c++){
            std::cout << std::setprecision(1) << salMap(r,c) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void vAttentionManager::decaySaliencyMap( yarp::sig::Matrix &salMap, unsigned long int dt) {
    double decayFactor = exp(-((double)dt)/tau);
//    std::cout << "exp(-dt/tau) = " << decayFactor << std::endl;
    for(int r = 0; r < sensorSize + filterSize - 1; r++) {
        for(int c = 0; c < sensorSize + filterSize - 1; c++) {
            salMap(r,c) = salMap(r,c) * decayFactor;
        }
    }

}

void vAttentionManager::normaliseSaliencyMap(yarp::sig::Matrix &salMap) {
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

    if (max == min){
        return;
    }

    // ---- normalise ---- //
    for(int r = 0; r < sensorSize; r++) {
        for(int c = 0; c < sensorSize; c++) {
            salMap(r,c) = (salMap(r,c) - min) / (max - min) ;
        }
    }
}

//empty line to make gcc happy
