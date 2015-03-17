
// Author: Himanshu Akolkar
 
// Code for getting receptive fields and applying them on iCub with DVS input.
 
 
#include "receptiveField.h"
#include <fstream>
 
using namespace yarp::os;
 
receptiveField::receptiveField()
{
 
// Initializing with parameters for a typical pyramidal cell
//size = 32;
fieldSizeX = 32;
fieldSizeY = 32;
valid = true;
weightVector.resize(fieldSizeX*fieldSizeY);
verbose = 1; 
 
    //ctor
}

receptiveField::receptiveField(std::string weightFile)
{
 
// Initializing with parameters for a typical pyramidal cell
//size = 32;
fieldSizeX = 32;
fieldSizeY = 32;
valid = true;
weightVector.resize(fieldSizeX*fieldSizeY);
verbose = 1; 
setWeightsFile(weightFile.c_str());
setWeights();
 
    //ctor
}

 
receptiveField::~receptiveField()
{
    //dtor
}
 
void receptiveField::setVerbose(bool value)
{
    verbose = value;
}


bool receptiveField::setFieldSize(int x, int y)
{
    fieldSizeX = x;
    fieldSizeY = y;
    
    if(verbose == 2)
    std::cout << "[Receptive field:] receptive field size set to" << fieldSizeX << " by " << fieldSizeY << std::endl;
    
    return true;   
}
 
int receptiveField::getFieldSizeX()
{
    return fieldSizeX;
}
 
int receptiveField::getFieldSizeY()
{
    return fieldSizeY;
}
 
bool receptiveField::normalizeWeights()
{

    if(verbose == 2)
    std::cout << "[Receptive field:] Normalizing weights" << std::endl;

    double sum = 0;
    for(int n=0; n < fieldSizeX*fieldSizeY; n++)
    {
        sum+=weightVector[n];
    }
     
     
    for(int n=0; n < fieldSizeX*fieldSizeY; n++)
    {
        weightVector[n] /= sum ;
    }
    return true;
     
}
 
bool receptiveField::multiplyWeights( double factor)
{

    if(verbose == 2)
    std::cout << "[Receptive field:] multiplying weights by " << factor << std::endl;

     
    for(int n=0; n < fieldSizeX*fieldSizeY; n++)
    {
        weightVector[n] *= factor ;
    }
     
}
   
bool receptiveField::isValid()
{
    if(verbose == 2)
    std::cout << "[Receptive field:] Receptive field object validity is " << valid << std::endl;

    return valid;
}
 
         
bool receptiveField::setWeights()
{

    if(verbose >= 1)
    {
        std::cout << "[Receptive field:] Setting weights: " << std::endl;
        std::cout << "                   Opening weight file " << weightsFile.c_str() << std::endl;
    }

    fid.open(weightsFile.c_str());

    valid = fid.is_open();

    if(verbose >= 1)
    {
        if(valid)
            std::cout << "[Receptive field:] Weights file opened succesfully" << std::endl;
        else
            {
                std::cout << "[Receptive field:] Weights file could not be opened" << std::endl;                
                return false;
            }
    }

    if(verbose >= 1)
    {
        std::cout << "[Receptive field:] Writing to vector" << std::endl;
    }
    
    for(int n=0; n < fieldSizeX*fieldSizeY; n++)
    {
        fid >> weightVector[n];
    }
    
return true;
}
 
bool receptiveField::setWeightsFile(std::string filename)
{
    weightsFile = filename;
    valid = true;
    return true;
}
 
double receptiveField::getWeightAt(int x, int y)
{
    if(x < 0 || y < 0 || x > fieldSizeX-1 || y > fieldSizeY-1)
    {
        return 0.0;
    }  
    return (weightVector[(x)*(fieldSizeY)+ y] );
}

