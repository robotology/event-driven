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


#define SIZE_OF_EVENT 2048 //8192  //default:8192 
// SIZE_OF_DATA = 8byte * SIZE_OF_EVENT

//#define SIZE_OF_DATA 1024
//#define SIZE_OF_DATA 2048
//#define SIZE_OF_DATA 4096
//#define SIZE_OF_DATA 8192
//#define SIZE_OF_DATA 9216
#define SIZE_OF_DATA 16384
//#define SIZE_OF_DATA 24576
//#define SIZE_OF_DATA 32768
//#define SIZE_OF_DATA 65536
//#define SIZE_OF_DATA 131072


//#define FAST
#define SLOW
//#define _DEBUG

#endif //CONFIG_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

