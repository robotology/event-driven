// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Charles Clercq
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
 * website: www.robotcub.org 
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

/**
 * @file convert.h
 * @brief A file where the config elements are stored
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <list>
#include <cstring>

#define MAX_THREAD 4
#define USE_PACKET_SEQUENCE_NUMBER 1

#define SIZE_RECT 15

//#define BUFFER_SIZE 8192
//#define BUFFER_SIZE 16384
//#define BUFFER_SIZE 24576
//#define BUFFER_SIZE 32768
#define BUFFER_SIZE 65536

#define CHUNKSIZE 32768 
#define TH1       32768  
#define TH2       65536
#define TH3       98304
#define BUFFERDIM 131702

//logpolar
#define TAU 1000
#define THRESHOLD 2
#define TMAX 1000

typedef struct s_AER_struct {
    int x;
    int y;
    int pol;
    unsigned int ts;
}AER_struct;

typedef struct s_cart_pos {
    int x;
    int y;
}cart_pos;

typedef struct s_GROUP {
    int id;
    std::list<AER_struct> levts;
    int size;
    int c_x;
    int c_y;
}t_GROUP;

typedef struct represCLE {
    short id;
    short xCog;
    short yCog;
    short channel;
    short numAE;
    short shapeType;
    short xSize;
    short ySize;
    unsigned long timestamp;
}reprCLE;

typedef struct represHGE {
    short x;
    short y;
    short radius;
    short channel;
    unsigned long timestamp;
}reprHGE;

#endif //__CONFIG_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

