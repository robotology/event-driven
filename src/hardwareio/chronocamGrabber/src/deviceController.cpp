/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it, chiara.bartolozzi@iit.it
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

#include "deviceController.h"
#include "deviceRegisters.h"

#include "ccam_device.h"
#include "i_ccam.h"
#include "i_events_stream.h"

#include "is_board_discovery_repository.h"
#include "i_biases.h"
#include "lib_atis.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>


vDevCtrl::vDevCtrl(std::string deviceName)
{

    this->deviceName = deviceName;
    biases = new AtisBiases();


}

bool vDevCtrl::connect()
{

    std::cout << "connecting" << std::endl;
    atis = Chronocam::IS_BoardDiscoveryRepository::open("", "");
     if (!atis) {
	    std::cerr << "Cannot open device" << std::endl;
        return false;
    } 
    cam = atis->get_facility<Chronocam::I_CCam>();

    cam->start();
    cam->reset();
    stream = atis->get_facility<Chronocam::I_EventsStream>();
    if(stream) {
        stream->start();
    }
    return true;

    // Open the device and flash firmware if requested

}

Chronocam::I_EventsStream &vDevCtrl::getStream()
{
    if (!stream) {
	std::cerr << "stream getter called before cam was initialized!" << std::endl;
    }
    return *this->stream;
}

void vDevCtrl::disconnect(bool andturnoff)
{
   return; 
}


bool vDevCtrl::configure(bool verbose)
{

    std::cout << "configuring!" << std::endl;
    if(!configureBiases())
        return false;
    std::cout << deviceName << ":" << " biases configured." << std::endl;
    if(verbose)
        printConfiguration();
    return true;
}


bool vDevCtrl::setBias(yarp::os::Bottle bias)
{
    if(bias.isNull())
        return false;

    this->bias = bias;
    return true;
}

// --- change the value of a single bias --- //
bool vDevCtrl::setBias(std::string biasName, unsigned int biasValue)
{
    yarp::os::Bottle &vals = bias.findGroup(biasName);
    if(vals.isNull()) return false;
    vals.pop(); //remove the old value
    vals.addInt(biasValue);
    return true;
}

unsigned int vDevCtrl::getBias(std::string biasName)
{
    yarp::os::Bottle &vals = bias.findGroup(biasName);
    if(vals.isNull()) return -1;
    return vals.get(3).asInt();
}

bool vDevCtrl::configureBiases(){


    suspend();

    
    // Flash biases if requested

    std::cout << "Programming " << bias.size() << " biases:" << std::endl;
    double vref, voltage;
    int header;
    int i;
    std::string toChange;

    for(i = 1; i < bias.size(); i++) {
        yarp::os::Bottle *biasdata = bias.get(i).asList();
        toChange = biasdata->get(0).asString();
        vref = biasdata->get(1).asInt();
        header = biasdata->get(2).asInt();
        voltage = biasdata->get(3).asInt();
        unsigned int biasVal = 255 * (voltage / vref);
        biasVal += header << 21;
        // bridging differences in the naming conventions
        if (toChange == "APSVrefL") toChange = "APSvrefL";
        if (toChange == "APSVrefH") toChange = "APSvrefH";
	    std::cout << i << " " << toChange << " " << voltage << std::endl;
    	biases->set(toChange, voltage);	
    }
    // Flash the biases
    Chronocam::I_Biases* i_biases = atis->get_facility<Chronocam::I_Biases>();
    i_biases->set_biases(biases);
    // --- checks --- //

    biases->to_file("/tmp/yarp_biases.txt");
    return activate(true);

}


bool vDevCtrl::suspend()
{
    return activate(false);
}


bool vDevCtrl::activate(bool active)
{
    if (active) {
    
        std::cout << "starting!" << std::endl;
        
        cam->start();
        cam->reset();
        // select if camera generate APS
        cam->set_couple(false);
    } else {
        if (cam) {
            cam->stop();
        }
    }

    return true;
}



void vDevCtrl::printConfiguration()
{

    std::cout << "Configuration for control device: " << deviceName << std::endl;

    std::cout << "== Bias Values ==" << std::endl;
    std::cout << bias.toString() << std::endl;


}




