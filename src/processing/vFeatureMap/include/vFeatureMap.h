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

#ifndef __VFEATMAP__
#define __VFEATMAP__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <math.h>

class vFeatureMapManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort; // /vBottle:o
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outFeaLeftOnPort;       // port whre the output feature map image (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outFeaLeftOffPort;       // port whre the output feature map image (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outFeaRightOnPort;      // port whre the output feature map image (right) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outFeaRightOffPort;      // port whre the output feature map image (right) is sent
    

    int* lut;                             // lut that route the event in a different location
    FILE *pFile;                          // file that contains the rules for the LUT
    FILE *fout;                           // file where the extracted LUT is saved
    //FILE *fdebug;                         // file for debug
    
    int retSize;
    int featSize;
    int scaleFactor;
    int thrOn;                      // threshold applied to the leftFeaOutputImageOn
    int thrOff;                     // threshold applied to the leftFeaOutputImageOn
    int constLeak;                  // constant leak of the feature maps
    
    std::string mapURL;             // mode name and name of the map
    //int rowSize;
    int rowSizeFea;
    int devianceFea;
    int devianceFeaSurround;

    //yarp::sig::ImageOf <yarp::sig::PixelMono>* leftOutputImage;         // image output left
    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftFeaOutputImageOn;    // output image of feature left center-on  response (registry)
    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftFeaOutputImageOff;   // output image of feature left center-off response (registry)
    yarp::sig::ImageOf <yarp::sig::PixelMono>* rightFeaOutputImageOn;    // output image of feature left center-on  response (registry)
    yarp::sig::ImageOf <yarp::sig::PixelMono>* rightFeaOutputImageOff;   // output image of feature left center-off response (registry)

    //for helping with timestamp wrap around
    //emorph::vtsHelper unwrapper;

    void updateFeatureMap(emorph::AddressEvent *aep, emorph::vBottle *outBottle);
    void leakFeatureMap(unsigned char* pFea);
    
public:

    vFeatureMapManager(int retSize, int featSize, int thrOn, int thrOff, int constLeak);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);
    
    /**
     * function that set operating mode
     * @param str name of the mode
     */
    void setMapURL(std::string str) { mapURL = str; };
    
};

class vFeatureMapModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vFeatureMapManager      *fmmanager;
    yarp::os::Port          handlerPort;                 // a port to handle messages


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

};


#endif
//empty line to make gcc happy
