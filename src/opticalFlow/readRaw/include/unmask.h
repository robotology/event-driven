#ifndef UNMASK_H
#define UNMASK_H

//#ifdef _DEBUG
#include <iostream>
#include <sstream>
#include <fstream>
//#endif

//Other dependency
#include "synapse.h"
//#define _DEBUG

class unmask
{
public:
	unmask();
//	unmask(void(*)(int, void**));
	~unmask();
    void unmaskData(char*, int, int);
private:
/***********
 * METHODS *
 **********/
	/**
	* @brief This method unmasked the raw which come from the TCP socket
	* This method have been wrote by the university of zurich. contact : tobi@ini.phys.ethz.ch
	* @param *evPU A pointer on the raw casted from char* to int*
	* @param x Set with the x coordinate of the pixel
	* @param y Set with the y coordinate of the pixel
	* @param pol Set with the ON/OFF polarity of the pixel.
	* @param ts ...
	*/
	void unmaskEvent(unsigned int, int&, int&, int&);
/*************
 * VARIABLES *
 ************/
	/*Variables needed by the unmask method*/
	int sz;
	char* buffer;
	unsigned int timestamp;
	int cartX, cartY, polarity;

    int wrapAdd;
	unsigned int xmask;
	unsigned int ymask;
	int yshift;
	int xshift;
	int polshift;
	int polmask;
	int retinalSize;

	unsigned int part_1;
	unsigned int part_2;
	unsigned int part_3;
	unsigned int part_4;
	unsigned int part_5;
	unsigned int part_6;
	unsigned int part_7;
	unsigned int part_8;

	unsigned int blob;
	//**************************************

    int sMapX;
    int sMapY;
    int neighLR;
    double stdDev;
    int threshold;
    int alpha;
    int tauC;
    int tauD;
    int accTime;
    int nApF;
    int selection;
//	void (*ptrFn)(int, void**);
//  void (*synapse::ptrFn)(int, void**);
    synapse objSynapse;
    void** argSYN;
    void** args;

//Compute time
    timespec start;
    timespec current;
    unsigned long int diff_time;
    std::stringstream ssBuffer;
    unsigned int oldT;
    unsigned int firstT;
    std::ofstream times;
};

#endif //UNMASK_H
