//
//  biasManager.cpp
//  eMorph
//
//  Created by Chiara Bartolozzi on 12/08/15.
//
//

#include "iCub/biasManager.h"
#include <string>
#include <iostream>

using namespace std;

biasManager::biasManager(){
    
    // todo: load names and values from config file (this way to change chip we only have to change the file)
    //tBiases.insert(TStrDoublePair("cas",52458));
    //tBiases.insert(TStrDoublePair("injg",101508));
//    tBiases.insert(TStrDoublePair("reqPd",16777215));
//    tBiases.insert(TStrDoublePair("pux",8053457));
//    tBiases.insert(TStrDoublePair("diffoff",133));
//    tBiases.insert(TStrDoublePair("req",160712));
//    tBiases.insert(TStrDoublePair("refr",944));
//    tBiases.insert(TStrDoublePair("puy",16777215));
//    tBiases.insert(TStrDoublePair("diffon",639172));
//    tBiases.insert(TStrDoublePair("diff",30108));
//    tBiases.insert(TStrDoublePair("foll",20));
//    tBiases.insert(TStrDoublePair("pr",5));
    
    tBiases["cas"] = 52458;
    tBiases["injg"] = 101508;
            
    
    /*
    biasNumber = 12;
    biasNames.resize(biasNumber);

    int i =0;
    // to be changed with file parsing
    biasNames[i] =  "cas";
    i++;
    biasNames[i] =  "injg";
    i++;
    biasNames[i] =  "reqPd";
    i++;
    biasNames[i] =  "pux";
    i++;
    biasNames[i] =  "diffoff";
    i++;
    biasNames[i] =  "req";
    i++;
    biasNames[i] =  "refr";
    i++;
    biasNames[i] =  "puy";
    i++;
    biasNames[i] =  "diffon";
    i++;
    biasNames[i] =  "diff";
    i++;
    biasNames[i] =  "foll";
    i++;
    biasNames[i] =  "pr";
    
    biasValues.resize(biasNumber);
    
    i = 0;
    
    biasValues[i] = 52458;         // cas
    i++;
    biasValues[i] = 101508;       // injGnd
    i++;
    biasValues[i] = 16777215;    // reqPd
    i++;
    biasValues[i] = 8053457;       // puX
    i++;
    biasValues[i] = 133;       // diffOff
    i++;
    biasValues[i] = 160712;        // req
    i++;
    biasValues[i] = 944;          // refr
    i++;
    biasValues[i] = 16777215;      // puY
    i++;
    biasValues[i] = 639172;     // diffOn
    i++;
    biasValues[i] = 30108;        // diff
    i++;
    biasValues[i] = 20;           // foll
    i++;
    biasValues[i] = 5;              // Pr
    */
    
    
}

bool biasManager::setBias(std::string biasName, double biasValue){
    
    int currBiasVal = tBiases[biasName];
    if (currBiasVal){
        tBiases[biasName] = biasValue;
        std::cout << "setting " << biasName.c_str() << " to value: " << tBiases[biasName] << std::endl;
        return true;
    } else {
        std::cout << "wrong bias name: please check spelling" << std::endl;
        return false;
    }
    
}

bool biasManager::progBias(){
    // implement here the transformation between the bias value list and the array of n bits that will be sent to the device
    
    TStrDoubleMap::iterator i;
    
    for (i = tBiases.begin(); i!= tBiases.end(); i++) {
        
        double bVal = i->second;
     
        /* progBias from aexGrabber
         int bitvalue;
         
         for (int i=bits-1;i>=0;i--) {
         int mask=1;
         for (int j=0; j<i; j++) {
         mask*=2;
         }
         //printf("mask %d ",mask);
         if (mask & value) {
         bitvalue = 1;
         }
         else {
         bitvalue = 0;
         }
         progBitAEs(bitvalue, camera); // sends a bit value per time
         
         */
        
    }
    
}






