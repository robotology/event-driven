//
//  biasManager.h
//  eMorph
//
//  Created by Chiara Bartolozzi on 12/08/15.
//
//

#ifndef __eMorph__biasManager__
#define __eMorph__biasManager__

#include <stdio.h>

#include <string>
#include <vector>
#include <map>


//typedef std::map<std::string, double> TStrDoubleMap;
//typedef std::pair<std::string, double> TStrDoublePair;


class biasManager{
    
    biasManager();
    
    int biasNumber;
    
    std::map<std::string, double> tBiases;
    
    //std::vector<std::string> biasNames;
    //std::vector<int> biasValues;
    
    //int findIndex();
    
    bool setBias(std::string biasName, double biasValue);

    bool progBias();
};




#endif /* defined(__eMorph__biasManager__) */
