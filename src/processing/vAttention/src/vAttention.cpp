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
    int filterSize = rf.check("filterSize", yarp::os::Value(1)).asInt();
    double tau = rf.check("tau", yarp::os::Value(120000.0)).asDouble();
    double thrSal = rf.check("thr", yarp::os::Value(30)).asDouble();

    /* create the thread and pass pointers to the module parameters */
    attManager = new vAttentionManager(sensorSize, filterSize, tau, thrSal);

    return attManager->open(moduleName, strict);

}

bool vAttentionModule::interruptModule() {
    attManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

bool vAttentionModule::close() {
    attManager->close();
    delete attManager;
    yarp::os::RFModule::close();
    return true;
}

bool vAttentionModule::updateModule() {
    return true;
}

double vAttentionModule::getPeriod() {
    return 1;
}

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

void vAttentionManager::load_filter(std::string filename, yarp::sig::Matrix &filterMap, int &filterSize) {

    //Opening filter file.
    ifstream file;
    file.open(filename.c_str(), ios::in | ios::out);
    if(!file.is_open()){
        std::cerr << "Could not open filter file " << filename << std::endl;
    }

    string line;
    int r = 0;
    int c = 0;

    //File is parsed line by line. Values are separated by spaces
    while(!std::getline(file, line, '\n').eof()) {
        istringstream reader(line);
        string::const_iterator i = line.begin();
        if (line.empty())
            continue;
        c = 0;

        while(!reader.eof()) {

            double val;
            reader >> val;

            //Resize the map to contain new values if necessary
            if (r + 1> filterMap.rows()){
                filterMap.resize(r + 1, filterMap.cols());
            }
            if (c + 1> filterMap.cols()){
                filterMap.resize(filterMap.rows(), c + 1);
            }

            filterMap(r,c) = val;
            c++;
        }
        r++;
    }

    //The returned filterSize is updated with the maximum dimension of the filter
    int maxDimension = std::max(filterMap.rows(),filterMap.cols());
    filterSize = max (filterSize,maxDimension);
}

vAttentionManager::vAttentionManager(int sensorSize, int filterSize, double tau, double thrSal) {
    this -> sensorSize = sensorSize;
    this -> tau = tau;
    this -> thrSal = thrSal;

    normSal = thrSal/255;

    ptime = 0; // past time stamp


    std::string filterDirectoryPath = "../../filters/";

    load_filter(filterDirectoryPath + "horizFilter.txt", horizFilterMap, filterSize);
//    load_filter(filterDirectoryPath + "vertFilter.txt", vertFilterMap, filterSize);
    vertFilterMap = horizFilterMap.transposed();
    load_filter(filterDirectoryPath + "uniformFilter.txt", uniformFilterMap, filterSize);
    uniformFilterMap *= 0.6;
    horizFilterMap *= 2;
    vertFilterMap *= 2;

    /** Gaussian Filter computation**/

    double sigma = 7;

    int gaussianFilterSize = 30;
    gaussianFilterMap.resize(gaussianFilterSize,gaussianFilterSize);
    for (int r = 0; r < gaussianFilterSize; r++){
        for (int c = 0; c < gaussianFilterSize; c++){
            double center = gaussianFilterSize / 2;
            double rDist = r - center;
            double cDist = c - center;
            gaussianFilterMap (r,c) = exp(-(pow(rDist,2)/(2*pow(sigma,2)) + pow(cDist,2)/(2*pow(sigma,2))));
        }
    }
    filterSize = max(gaussianFilterSize,filterSize);
    gaussianFilterMap *= 0.5;

    printSaliencyMap(uniformFilterMap);
    printSaliencyMap(vertFilterMap);
    printSaliencyMap(horizFilterMap);
    printSaliencyMap(gaussianFilterMap);


    this->filterSize = filterSize;
    this -> salMapPadding = filterSize/2;

    //for speed we predefine the memory for some matrices
    //The saliency map is bigger than the image by the maximum size among the loaded filters
    salMapLeft  = yarp::sig::Matrix(sensorSize+filterSize, sensorSize+filterSize);
    salMapRight = yarp::sig::Matrix(sensorSize+filterSize, sensorSize+filterSize);

    // initialise saliency map to zero
//    salMapLeft.zero();
//    salMapRight.zero();

    salMapLeft = 1;
    salMapRight = 1;

}

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

//    salMapImageLeft   = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
//    salMapImageRight  = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
//
//    salMapImageLeft ->resize(sensorSize,sensorSize);
//    salMapImageRight->resize(sensorSize,sensorSize);
//
//    salMapImageLeft ->zero();
//    salMapImageRight->zero();

    // ---- initialise the images of the saliency maps left and right to 0 ---- //
    //memset((void*)salMapImageLeft, 0, sensorSize * sensorSize * sizeof(unsigned char));
    //memset((void*)salMapImageRight, 0, sensorSize * sensorSize * sizeof(unsigned char));

    std::cout << "initialisation correctly ended" << std::endl;

    return check1 && check2 && check3 && check4;
}

void vAttentionManager::close() {
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
    outSalMapLeftPort.close();
    outSalMapRightPort.close();

    //free(salMapImageLeft);
    //free(salMapImageRight);
}

void vAttentionManager::interrupt() {
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    outSalMapLeftPort.interrupt();
    outSalMapRightPort.interrupt();
}

void vAttentionManager::onRead(emorph::vBottle &bot) {
    /* get the event queue in the vBottle bot */
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.sort(true);

    unsigned long int t  = unwrap(q.back()->getStamp());
    unsigned long int dt  = t - ptime;
    ptime = t;

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        // --- increase energy of saliency map  --- //
        if (aep->getChannel() == 0) {
            updateSaliencyMap(salMapLeft, vertFilterMap, aep);
            updateSaliencyMap(salMapLeft, horizFilterMap, aep);
            updateSaliencyMap(salMapLeft, gaussianFilterMap, aep);
        }
        else {
            updateSaliencyMap(salMapRight, horizFilterMap, aep);
        }
    }

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
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st);
    outPort.setEnvelope(st);
    // --- writing vBottle on buffered output port
    if (strictness) {
        outPort.writeStrict();
    } else {
        outPort.write();
    }
     */

    //  --- convert to images for display --- //


    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageLeft = outSalMapLeftPort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageRight = outSalMapRightPort.prepare();

    convertToImage(salMapLeft, imageLeft);
    convertToImage(salMapRight, imageRight);

    // --- writing images of left and right saliency maps on output port
    if(outSalMapLeftPort.getOutputCount()) {
        outSalMapLeftPort.write();
    }
    if(outSalMapRightPort.getOutputCount()) {
        outSalMapRightPort.write();
    }
}

void vAttentionManager::convertToImage(yarp::sig::Matrix &salMap, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) {

    /*prepare output vBottle with images */
    image.resize(sensorSize,sensorSize);
    image.setTopIsLowIndex(true);
    image.zero();

    double* attentionPoint = computeAttentionPoint(salMap);

    for(int r = sensorSize; r > 0; r--) {
        for(int c = 0; c < sensorSize; c++) {
            yarp::sig::PixelBgr pixelBgr;

//            Coordinates of saliency map are shifted by salMapPadding wrt the image
            double pixelValue = std::min(salMap(r + salMapPadding, c + salMapPadding), thrSal);

            //Normalize to maximum pixel bgr value 255
            pixelValue /= normSal;

            //Attention point is highlighted in red, negative values in blue, positive in green
            if (&salMap(r,c) == attentionPoint) {
                pixelBgr.r = 255;
            }
            else if (pixelValue <= 0) {
//                pixelBgr.b = std::min (fabs(pixelValue),255.0);
            }else {
                pixelBgr.g = std::min (fabs(pixelValue),255.0);
                }

            image(c,sensorSize - r) = pixelBgr;
        }
    }
}

void vAttentionManager::updateSaliencyMap(yarp::sig::Matrix &salMap, yarp::sig::Matrix &filterMap, emorph::AddressEvent *aep) {
    //Pixel coordinates are shifted to match with the location in the saliency map

    int filterRows = filterMap.rows();
    int filterCols = filterMap.cols();

    // unmask event: get x, y
    int r = aep->getX() + salMapPadding;
    int c = aep->getY() + salMapPadding;

    // ---- increase energy in the location of the event ---- //

//    int rows = salMap.rows();
//    int cols = salMap.cols();

    for(int rf = -filterRows/2; rf < filterRows/2; rf++) {
        for(int cf = -filterCols/2; cf < filterCols/2; cf++) {
            salMap(r, c) += fabs(salMap(r + rf, c + cf)) * filterMap(rf + filterRows/2,cf + filterCols/2);
            salMap(r,c) = min(salMap(r, c), 2000.0);
            salMap(r,c) = max(salMap(r, c), -2000.0);
            cf ++;
        }
        rf ++;
    }
}

void vAttentionManager::printSaliencyMap (yarp::sig::Matrix &salMap) {
    for (int r = 0; r < salMap.rows(); r ++){
        for (int c = 0; c < salMap.cols(); c++){
            std::cout << setprecision(2) << salMap(r,c) << " ";
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

double* vAttentionManager::computeAttentionPoint(yarp::sig::Matrix &salMap){
    double max = 0;
    int rMax = 0;
    int cMax = 0;
    for(int r = 0; r < sensorSize; r++) {
        for(int c = 0; c < sensorSize; c++) {
            if (salMap(r,c) > max){
                max = salMap(r,c);
                rMax = r;
                cMax = c;
            }
        }
    }
    return &salMap(rMax,cMax);
}

//empty line to make gcc happy
