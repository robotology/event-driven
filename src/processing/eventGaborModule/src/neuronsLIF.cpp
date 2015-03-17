
#include "neuronsLIF.h"
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>

neuronLIF::neuronLIF()
{
// Init neuron related variables
	restingPotential = 0;
	potential = restingPotential;
    thresholdPotential = 10;
	lastTimeStamp = 0;
	lastUpdateTime = 0;
	timeFromLastSpike = 0;
	totalSpikes = 0;
	initTimeStamp = 0;
//	inputCurrent = 0;
	membraneTimeConstant = 10;
	
	setInputCurrent();
// Init display parameters
	display = false;
	
// Init file save parameters	
	updated = false;
	save2file = false;
	neuronId = "defaultNeuron";
    fileName = neuronId + ".dat";
    //saveFid.is_open();
    
    if (save2file)
       saveFid.open(fileName.c_str());  

// Debug ??
   debug = true;
}

neuronLIF::neuronLIF(neuronLIF& n)
{
    potential = n.potential;
    restingPotential = n.restingPotential;	          
	n.thresholdPotential = n.thresholdPotential;
    n.centerX = n.centerX;
    n.centerY = n.centerY;

              
    n.lastTimeStamp = n.lastTimeStamp;
	n.lastUpdateTime = n.lastUpdateTime;
	n.timeFromLastSpike = n.timeFromLastSpike;
    n.totalSpikes = n.totalSpikes;
    n.initTimeStamp = n.initTimeStamp;
	n.inputCurrent = n.inputCurrent;
	n.membraneTimeConstant = n.membraneTimeConstant;
              // Init display parameters
	n.display = n.display;
	          // Init file save parameters	
    n.updated = n.updated;
                //n.save2file = this->save2file;
    n.fileName = n.fileName;              
    n.neuronId = n.neuronId;
              
    n.debug = n.debug;
                //n.saveFid = this->saveFid;
  
};


neuronLIF::~neuronLIF()
{
       saveFid.close();                
}

//neuronLIF neuronLIF::operator=(neuronLIF& n)
//{

//        n.restingPotential = this->restingPotential;
//}

bool neuronLIF::updateNeuron()
{
     if (debug)
        std::cout << "[neuron LIF]: Dummy function called" << std::endl;

     double curr = 0;
     if (curr <= 0)
     {
              return false;
     }
     
     potential = potential; //*exp(-abs(delT)/membraneTimeConstant) + curr;
     lastUpdateTime = time(0);
     
     if(potential > thresholdPotential)
     {
                  potential = restingPotential;
                  lastTimeStamp = lastUpdateTime;
                  totalSpikes++;
                  if(save2file) writeToFile();
                  return true;
     }
     else
     {     
           if(save2file) writeToFile();
           return false;
     }
}

bool neuronLIF::updateNeuron(double curr, unsigned int timeNow)
{
     if (debug)
          std::cout << "[neuron LIF]: Update neurons with current " << curr << " and time " << lastUpdateTime << " " << timeNow << " " << timeNow - lastUpdateTime<< "  " << std::endl;
     
     double delT = timeNow - lastUpdateTime;
     if (delT < 0) delT = - delT;
             
     potential = potential*exp(-(delT/membraneTimeConstant) ) + curr;
     lastUpdateTime = timeNow;
     
     if(potential <= restingPotential)
     {
            potential = restingPotential;
            return false;
     }  

     if(potential > thresholdPotential)
     {
                  potential = restingPotential;
                  lastTimeStamp = lastUpdateTime;
                  totalSpikes++;
                  if(save2file) writeToFile();
                  return true;
     }
     else
     {     
           if(save2file) writeToFile();
           return false;
     }
}

bool neuronLIF::updateNeuron(double curr, double delT)
{
     if (debug)
        std::cout << "[neuron LIF]: Update neuron with current and delT" << std::endl;
     
     if (delT < 0) delT = - delT;

     potential = potential*exp(-(delT/membraneTimeConstant)) + curr;
     lastUpdateTime = lastUpdateTime + (unsigned int)delT;

     if(potential <= restingPotential)
     {
            potential = restingPotential;
            return false;
     }  

     if(potential > thresholdPotential)
     {
                  potential = restingPotential;
                  lastTimeStamp = lastUpdateTime;
                  totalSpikes++;
                  if(save2file) writeToFile();
                  return true;
     }
     else
     {     
           if(save2file) writeToFile();
           return false;
     }

}


bool neuronLIF::updateNeuron(double curr)
{

     if (debug)
          std::cout << "[neuron LIF]: Update neurons with current: " << curr << std::endl;

     //potential = potential*exp(-1/membraneTimeConstant) + curr;
     potential = potential*0.96 + curr;
     lastUpdateTime = time(0);

     if (debug)
        std::cout << "[neuron LIF]: Update potential of neuron to " << potential << std::endl; 
     
     if(potential <= restingPotential)
     {
            potential = restingPotential;
            return false;
     }  

     if(potential > thresholdPotential)
     {
                  potential = restingPotential;
                  lastTimeStamp = lastUpdateTime;
                  totalSpikes++;
                  if(save2file) writeToFile();
                  return true;
     }
     else
     {     
           if(save2file) writeToFile();
           return false;
     }
     
}

void neuronLIF::setNeuronCenter(double u, double v)
{
     centerX = u;
     centerY = v;
}

double neuronLIF::getCenterX()
{
        return centerX;
}

double neuronLIF::getCenterY()
{
        return centerY;
}

unsigned int neuronLIF::getNumSpikes()
{
        return totalSpikes;
}

void neuronLIF::save2File(bool value)
{
     if(debug)
     std::cout << "[neuron LIF]: Setting write option to " << value << std::endl;
     save2file = value;     
}

void neuronLIF::setDebug( bool value)
{
     debug = value;
}

unsigned int neuronLIF::getTimeStamp()
{
	return lastTimeStamp;
}

double neuronLIF::getPotential()
{
	return potential;
}	
	
void neuronLIF::setInputCurrent()
{
     inputCurrent = 0;
}	

bool neuronLIF::setInputCurrent(double curr)
{
     inputCurrent = curr;
     return true;
}

bool neuronLIF::setThresholdPotential(double threshold)
{
     thresholdPotential = threshold;
     return true;
}

void neuronLIF::writeToFile()
{
     if(debug)
     std::cout << "[neuron LIF]: writing to File" << std::endl;
     
     if(!saveFid.is_open())
                saveFid.open(fileName.c_str());
                
     if(save2file)
          saveFid << lastUpdateTime << " " << potential << " " << lastTimeStamp << std::endl;
}        

bool neuronLIF::setNeuronId(std::string value)
{
     neuronId = value;
     fileName = neuronId + ".dat";
     return true;
}

void neuronLIF::resetNeuron()
{
//    restingPotential = 0;
//    potential = 0;
    totalSpikes = 0;

}
