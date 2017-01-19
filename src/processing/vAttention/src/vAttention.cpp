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
#include <fstream>

using namespace yarp::math;
using namespace std;

bool vAttentionModule::configure(yarp::os::ResourceFinder &rf) {
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vAttention")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strictness = rf.check("strictness") &&
                  rf.check("strictness", yarp::os::Value(true)).asBool();

    /* attach a port of the same name as the module (prefixed with a /) to the module
     so that messages received from the port are redirected to the respond method */

    std::string handlerPortName = "/";
    handlerPortName += getName();         // use getName() rather than a literal

    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }

    attach(handlerPort);                  // attach to port

    /* set parameters */
    int sensorSize = rf.check("sensorSize", yarp::os::Value(128)).asInt();
    double tau = rf.check("tau", yarp::os::Value(200000.0)).asDouble();
    double thrSal = rf.check("thr", yarp::os::Value(20)).asDouble();
    string filtersPath = rf.check("filtersPath", yarp::os::Value("../../src/processing/vAttention/filters/")).asString();

    /* create the thread and pass pointers to the module parameters */
    attManager = new vAttentionManager(sensorSize, tau, thrSal, filtersPath);

    return attManager->open(moduleName, strictness);

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

bool vAttentionModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply) {
    std::string helpMessage = std::string(getName().c_str()) +
                              " commands are: \n" +
                              "help \n" +
                              "quit \n";
    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    } else if (command.get(0).asString() == "help") {
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
    if (!file.is_open()) {
        std::cerr << "Could not open filter file " << filename << std::endl;
    }

    string line;
    int r = 0;
    int c = 0;

    //File is parsed line by line. Values are separated by spaces
    while (!std::getline(file, line, '\n').eof()) {
        istringstream reader(line);
        string::const_iterator i = line.begin();
        if (line.empty())
            continue;
        c = 0;

        while (!reader.eof()) {

            double val;
            reader >> val;

            //Resize the map to contain new values if necessary
            if (r + 1 > filterMap.rows()) {
                filterMap.resize(r + 1, filterMap.cols());
            }
            if (c + 1 > filterMap.cols()) {
                filterMap.resize(filterMap.rows(), c + 1);
            }

            filterMap(r, c) = val;
            c++;
        }
        r++;
    }

    //The returned filterSize is updated with the maximum dimension of the filter
    int maxDimension = std::max(filterMap.rows(), filterMap.cols());
    filterSize = max(filterSize, maxDimension);
}

vAttentionManager::vAttentionManager(int sensorSize, double tau, double thrSal, std::string &filtersPath) {
    this->sensorSize = sensorSize;
    this->tau = tau;
    this->thrSal = thrSal;

    normSal = thrSal / 255;

    ptime = 0; // past time stamp



    load_filter(filtersPath + "/horizFilter.txt", horizFilterMap, filterSize);
    load_filter(filtersPath + "/bigHorizFilter.txt", bigHorizFilterMap, filterSize);
//    load_filter(filterDirectoryPath + "vertFilter.txt", vertFilterMap, filterSize);
    vertFilterMap = horizFilterMap.transposed();
    bigVertFilterMap = bigHorizFilterMap.transposed();
    load_filter(filtersPath + "/uniformFilter.txt", uniformFilterMap, filterSize);

    /** Gaussian Filter computation**/

    generateGaussianFilter(gaussianFilterMap, 2, 20, filterSize);
    generateGaussianFilter(bigGaussianFilterMap, 2.5, 20, filterSize);

    DOGFilterMap = bigGaussianFilterMap - gaussianFilterMap;
    DOGFilterMap *= 150;
    printMap(DOGFilterMap);
    this->salMapPadding = filterSize / 2;

    //for speed we predefine the memory for some matrices
    //The saliency map is bigger than the image by the maximum size among the loaded filters
    salMapLeft = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    salMapRight = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    vertFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    bigVertFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    horizFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    bigHorizFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    gaussianFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    bigGaussianFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);
    DOGFeatureMap = yarp::sig::Matrix(sensorSize + filterSize, sensorSize + filterSize);

    // initialise saliency map to zero
//    salMapLeft.zero();
//    salMapRight.zero();

    salMapLeft = 1;
    salMapRight = 1;
    vertFeatureMap = 1;
    bigVertFeatureMap = 1;
    horizFeatureMap = 1;
    bigHorizFeatureMap = 1;
    gaussianFeatureMap = 1;
    bigGaussianFeatureMap = 1;
    DOGFeatureMap = 1;

}


void vAttentionManager::generateGaussianFilter(yarp::sig::Matrix &filterMap, double sigma, int gaussianFilterSize, int &filterSize) {
    //Resize to desired size
    filterMap.resize(gaussianFilterSize, gaussianFilterSize);

    //Generate gaussian filter
    for (int r = 0; r < gaussianFilterSize; r++) {
        for (int c = 0; c < gaussianFilterSize; c++) {
            double center = gaussianFilterSize / 2;
            double rDist = r - center;
            double cDist = c - center;
            filterMap(r, c) =(1/(sigma * sqrt(2*M_PI)))* exp(-(pow(rDist, 2) / (2 * pow(sigma, 2)) + pow(cDist, 2) / (2 * pow(sigma, 2))));
        }
    }
    //Update filterSize to comply with new filter
    filterSize = max(gaussianFilterSize, filterSize);
}

bool vAttentionManager::open(const std::string moduleName, bool strictness) {
    this->strictness = strictness;
    if (strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    // why is the input port treated differently???? both in open and close
    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    if (strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    outPortName = "/" + moduleName + "/salMapLeft:o";
    bool check3 = outSalMapLeftPort.open(outPortName);

    outPortName = "/" + moduleName + "/salMapRight:o";
    bool check4 = outSalMapRightPort.open(outPortName);

    std::cout << "opened ports: " << std::endl << "vBottle:i " << check1 << std::endl << "vBottle:o " << check2
              << std::endl << "/salMapLeft:o " << check3 << std::endl << "/salMapRight:o " << check4 << std::endl;

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

    unsigned long int t = unwrap(q.back()->getStamp());
    unsigned long int dt = t - ptime;
    ptime = t;

    for (emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if (!aep) continue;

        // --- increase energy of saliency map  --- //
        if (aep->getChannel() == 0) {
//            updateMap(vertFeatureMap, vertFilterMap, aep);
//            updateMap(bigVertFeatureMap, vertFilterMap, aep);
//            updateMap(horizFeatureMap, horizFilterMap, aep);
//            updateMap(bigHorizFeatureMap, bigHorizFilterMap, aep);
//            updateMap(gaussianFeatureMap, gaussianFilterMap, aep);
//            updateMap(bigGaussianFeatureMap, bigGaussianFilterMap, aep);
            updateMap(DOGFeatureMap,DOGFilterMap, aep);

        } else {
            updateMap(salMapRight, horizFilterMap, aep);
        }
    }
//printMap(vertFeatureMap);
//    decayMap(vertFeatureMap,dt);
//    decayMap(bigVertFeatureMap,dt);
//    decayMap(horizFeatureMap,dt);
//    decayMap(bigHorizFeatureMap,dt);
//    decayMap(gaussianFeatureMap,dt);
//    decayMap(bigGaussianFeatureMap,dt);
    decayMap(DOGFeatureMap, dt);
//    normaliseMap(vertFeatureMap, normalisedVertFeatureMap);
//    normaliseMap(bigVertFeatureMap, normalisedVertFeatureMap);
//    normaliseMap(horizFeatureMap, normalisedHorizFeatureMap);
//    normaliseMap(bigHorizFeatureMap, normalisedBigHorizFeatureMap);
//    normaliseMap(gaussianFeatureMap, normalisedGaussianFeatureMap);
//    normaliseMap(bigGaussianFeatureMap, normalisedBigGaussianFeatureMap);
//    normaliseMap(DOGFeatureMap, normalisedGaussianFeatureMap);
    //salMapLeft = normalisedVertFeatureMap + 5*normalisedGaussianFeatureMap + normalisedHorizFeatureMap;
    salMapLeft = DOGFeatureMap;

//    salMapLeft = 2*(normalisedBigGaussianFeatureMap+normalisedGaussianFeatureMap) + vertFeatureMap + bigVertFeatureMap + horizFeatureMap + bigHorizFeatureMap;
//    salMapLeft = bigHorizFeatureMap + horizFeatureMap;
//    salMapLeft *= 20000;

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
    if (outSalMapLeftPort.getOutputCount()) {
        outSalMapLeftPort.write();
    }
    if (outSalMapRightPort.getOutputCount()) {
        outSalMapRightPort.write();
    }
}

void vAttentionManager::convertToImage(yarp::sig::Matrix &map, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) {

    /*prepare output vBottle with images */
    image.resize(sensorSize, sensorSize);
    image.setTopIsLowIndex(true);
    image.zero();

    double *attentionPoint = computeAttentionPoint(map);

    for (int r = sensorSize; r > 0; r--) {
        for (int c = 0; c < sensorSize; c++) {
            yarp::sig::PixelBgr pixelBgr;

//            Coordinates of saliency map are shifted by salMapPadding wrt the image
//            double pixelValue = std::min(map(r + salMapPadding, c + salMapPadding), thrSal);
//
//            //Normalize to maximum pixel bgr value 255
//            pixelValue /= normSal;
            double pixelValue = map(r + salMapPadding, c + salMapPadding);
            //Attention point is highlighted in red, negative values in blue, positive in green
            if (&map(r + salMapPadding, c + salMapPadding) == attentionPoint) {
                pixelBgr.r = 255;
                drawSquare(image, r + salMapPadding, c + salMapPadding, pixelBgr);
            } else if (pixelValue <= 0) {
                pixelBgr.b = std::min (fabs(pixelValue),255.0);
            } else {
                pixelBgr.g = (unsigned char)std::min(fabs(pixelValue), 255.0);
            }

            image(c, sensorSize - r) = pixelBgr;
        }
    }
}

void vAttentionManager::drawSquare( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int r, int c,
                                   yarp::sig::PixelBgr &pixelBgr)  {
    int squareSize = 2;
    for (int i = -squareSize; i <= squareSize; ++i) {
                    for (int j = -squareSize; j <= squareSize; ++j) {
                        if ((r + i< (image.height() -1)) && r + i>= 0)
                            if ((c +j< (image.width() -1)) && c + j>= 0)
                                image(r +i,c+j) = pixelBgr;
                    }
                }
}

void vAttentionManager::updateMap(yarp::sig::Matrix &map, yarp::sig::Matrix &filterMap,
                                  emorph::AddressEvent *aep) {
    //Pixel coordinates are shifted to match with the location in the saliency map
//    printMap(map);
    int filterRows = filterMap.rows();
    int filterCols = filterMap.cols();

    // unmask event: get x, y
    int r = aep->getX() + salMapPadding;
    int c = aep->getY() + salMapPadding;
double val;
    // ---- increase energy in the location of the event ---- //

    for (int rf = -filterRows / 2; rf < filterRows / 2 + filterRows % 2; rf++) {
        for (int cf = -filterCols / 2; cf < filterCols / 2 + filterCols % 2; cf++) {
//            val = fabs(map(r + rf, c + cf)) * filterMap(rf + filterRows / 2, cf + filterCols / 2);
//            map(r, c) += val;
//            map(r, c) = min(map(r, c), 2000.0);
//            map(r, c) = min(map(r, c), 2000.0);
            map(r + rf, c + cf) += filterMap(rf + filterRows /2, cf + filterCols /2);
            map(r + rf, c + cf) = min(map(r + rf, c + cf), 2000.0);
            map(r + rf, c + cf) = max(map(r + rf, c + cf), -2000.0);

        }
    }
//    printMap(map);
}

void vAttentionManager::printMap(yarp::sig::Matrix &map) {
    for (int r = 0; r < map.rows(); r++) {
        for (int c = 0; c < map.cols(); c++) {
            std::cout << setprecision(2) << map(r, c) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void vAttentionManager::decayMap(yarp::sig::Matrix &map, unsigned long int dt) {
    double decayFactor = exp(-((double) dt) / tau);
    map*= decayFactor;
}

//void vAttentionManager::normaliseMap(yarp::sig::Matrix &map) {
//    double min;
//    double max;
//
//    min = map(0, 0);
//    max = min;
//
//    // ---- find max and min values of saliency map ---- //
//    for (int r = 0; r < map.rows(); r++) {
//        for (int c = 0; c < map.cols(); c++) {
//            if (map(r, c) > max) {
//                max = map(r, c);
//            }
//            if (map(r, c) < min) {
//                min = map(r, c);
//            }
//        }
//    }
//
//    if (max == min) {
//        return;
//    }
//    double value;
//    double salValue;
//    // ---- normalise ---- //
//    for (int r = 0; r < map.rows(); r++) {
//        for (int c = 0; c < map.cols(); c++) {
//            salValue = map(r,c);
//            value = (map(r, c) - min) / (max - min);
//            map(r, c) = value;
//        }
//    }
//}


void vAttentionManager::normaliseMap(yarp::sig::Matrix &map, yarp::sig::Matrix &normalisedMap) {
    double totalEnergy;
    normalisedMap = map;
    for (int r = 0; r < map.rows(); ++r) {
        for (int c = 0; c < map.cols(); ++c) {
            totalEnergy += map(r,c);
        }
    }
    normalisedMap /= totalEnergy;
}

double *vAttentionManager::computeAttentionPoint(yarp::sig::Matrix &map) {
    double max = 0;
    int rMax = 0;
    int cMax = 0;
    for (int r = 0; r < map.rows(); r++) {
        for (int c = 0; c < map.cols(); c++) {
            if (map(r, c) > max) {
                max = map(r, c);
                rMax = r;
                cMax = c;
            }
        }
    }
    map(rMax,cMax) *= 3;
    return &map(rMax, cMax);
}

//empty line to make gcc happy
