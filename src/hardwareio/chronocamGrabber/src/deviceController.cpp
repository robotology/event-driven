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

#include "lib_atis.h"
#include "lib_atis_biases.h"
#include "lib_atis_instance.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>


vDevCtrl::vDevCtrl(std::string deviceName)
{

    this->deviceName = deviceName;
    biases = new ATIS_EXPORT::AtisBiases();


}

bool vDevCtrl::connect()
{

    atis = new Atis();

    // Open the device and flash firmware if requested
    cam = atis->open_atis_auto("","");
    if (cam->get_last_err() != 0) {
	std::cerr << "Cannot open device:" << (int) cam->get_last_err() << std::endl;
	return false;
    } 

    return true;

}

AtisInstance &vDevCtrl::getCam()
{
    if (!cam) {
	std::cerr << "cam getter called before cam was initialized!" << std::endl;
    }
    return *this->cam;
}

void vDevCtrl::disconnect(bool andturnoff)
{
   return; 
}


bool vDevCtrl::configure(bool verbose)
{

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

    for(i = 1; i < bias.size(); i++) {
        yarp::os::Bottle *biasdata = bias.get(i).asList();
        vref = biasdata->get(1).asInt();
        header = biasdata->get(2).asInt();
        voltage = biasdata->get(3).asInt();
        unsigned int biasVal = 255 * (voltage / vref);
        biasVal += header << 21;
        //std::cout << biasdata->get(0).asString() << " " << biasVal << std::endl;
	AtisBiases::Bias toChange;
    	switch(i) {
		case 1:  toChange = AtisBiases::CtrlbiasLP       ; break;
		case 2:  toChange = AtisBiases::CtrlbiasLBBuff   ; break;
		case 3:  toChange = AtisBiases::CtrlbiasDelTD    ; break;
		case 4:  toChange = AtisBiases::CtrlbiasSeqDelAPS; break;
		case 5:  toChange = AtisBiases::CtrlbiasDelAPS   ; break;
		case 6:  toChange = AtisBiases::biasSendReqPdY   ; break;
		case 7:  toChange = AtisBiases::biasSendReqPdX   ; break;
		case 8:  toChange = AtisBiases::CtrlbiasGB       ; break;
		case 9:  toChange = AtisBiases::TDbiasReqPuY     ; break;
		case 10: toChange = AtisBiases::TDbiasReqPuX     ; break;
		case 11: toChange = AtisBiases::APSbiasReqPuY    ; break;
		case 12: toChange = AtisBiases::APSbiasReqPuX    ; break;
		case 13: toChange = AtisBiases::APSvrefL         ; break;
		case 14: toChange = AtisBiases::APSvrefH         ; break;
		case 15: toChange = AtisBiases::APSbiasOut       ; break;
		case 16: toChange = AtisBiases::APSbiasHyst      ; break;
		case 17: toChange = AtisBiases::APSbiasTail      ; break;
		case 18: toChange = AtisBiases::TDbiasCas        ; break;
		case 19: toChange = AtisBiases::TDbiasInv        ; break;
		case 20: toChange = AtisBiases::TDbiasDiffOff    ; break;
		case 21: toChange = AtisBiases::TDbiasDiffOn     ; break;
		case 22: toChange = AtisBiases::TDbiasDiff       ; break;
		case 23: toChange = AtisBiases::TDbiasFo         ; break;
		case 24: toChange = AtisBiases::TDbiasRefr       ; break;
		case 25: toChange = AtisBiases::TDbiasPR         ; break;
		case 26: toChange = AtisBiases::TDbiasBulk       ; break;
		default: continue;
	}
	std::cout << i << " " << toChange << " " << voltage << std::endl;
    	biases->set(toChange, voltage);	
    }
    // Flash the biases
    cam->set_biases(biases); //TODO: fix this.
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
    
        cam->start();
        cam->reset();
        // select if camera generate APS
        cam->set_couple(false);
    } else {
        cam->stop();
    }

    return cam->is_running()==active;
}



void vDevCtrl::printConfiguration()
{

    std::cout << "Configuration for control device: " << deviceName << std::endl;

    std::cout << "== Bias Values ==" << std::endl;
    std::cout << bias.toString() << std::endl;


}




