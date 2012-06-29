/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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

#ifndef PARAM_H_
#define PARAM_H_

#define LUCAS_KANADE_NGHBR 3
#define LUCAS_KANADE_GUAS_STDDEV 1.5

//+++++++++++++++++++++++++++++++++++++++++++++++++++

#define TS_RANGE 0x04000000  // There is 26 bits to represent timestamp
#define SPATIAL_MARGINE_ADDUP 4  // Spatial Window + LK window

#define SPDerivative_WNDW_SZ 10

#define POLARITY_WEIGHT  1

// For ICub Sensor
#define RELIABLE_EVENT_THRSHLD 12500// 93750 // 12500 //   31250 //  18750 // 6250 // 3125 //
// For DVS Sensor
//#define RELIABLE_EVENT_THRSHLD 3000

#define RELIABLE_NGHBRHD  1


//+++++++++++++++++++++++++++++++++++++++++++++++++++

#define POLARITY_TYPE int
#define TIMESTAMP_TYPE unsigned long

//+++++++++++++++++++++++++++++++++++++++++++++++++++
#define RETINA_SIZE_R 128
#define RETINA_SIZE_C 128

#endif /* PARAM_H_ */
