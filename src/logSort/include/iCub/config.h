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
 * @file config.h
 * @brief Simple header file for configuration. Intentionally separated for estetic purposes.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <list>
#include <cstring>
#include <inttypes.h>

#define MAX_THREAD 4
#define USE_PACKET_SEQUENCE_NUMBER 1
#define SIZE_RECT 15

//#define SIZE_OF_DATA  32768 // SIZE_OF_DATA = 8byte * SIZE_OF_EVENT

#define EVENTS1024            // defines the dimension of the buffers

#ifdef EVENTS4096
 #define SIZE_OF_EVENT 4096   //default:8192  CHUNKSIZE / 8  
 #define CHUNKSIZE     32768 
 #define TH1           32768  
 #define TH2           65536
 #define TH3           98304
 #define BUFFERDIM     131702
#endif
    
#ifdef EVENTS1024
 #define SIZE_OF_EVENT 1024   //default:8192  CHUNKSIZE / 8  
 #define CHUNKSIZE     8192 
 #define TH1           8192  
 #define TH2           16384
 #define TH3           24576
 #define BUFFERDIM     32768
#endif

#ifdef EVENTS512
 #define SIZE_OF_EVENT 512   //default:8192  CHUNKSIZE / 8  
 #define CHUNKSIZE     4096 
 #define TH1           4096  
 #define TH2           8192
 #define TH3           12288
 #define BUFFERDIM     16384
#endif

#ifdef EVENTS256
 #define SIZE_OF_EVENT 256   //default:8192  CHUNKSIZE / 8  
 #define CHUNKSIZE     2048 
 #define TH1           2048  
 #define TH2           4096
 #define TH3           6144
 #define BUFFERDIM     8192
#endif


#ifdef EVENTS128
 #define SIZE_OF_EVENT 128   //default:8192  CHUNKSIZE / 8  
 #define CHUNKSIZE     1024 
 #define TH1           1024  
 #define TH2           2048
 #define TH3           3072
 #define BUFFERDIM     4096
#endif



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

#define u32 uint32_t

struct aer {
    u32 address;
    u32 timestamp;
};

//#define FAST
#define SLOW
//#define _DEBUG

#endif //CONFIG_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

