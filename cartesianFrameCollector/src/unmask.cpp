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
 * @file unmask.cpp
 * @brief A class for unmasking the event (see the header unmask.h)
 */

#include <iCub/unmask.h>
using namespace std;

unmask::unmask() {
    xmask = 0x000000fE;
    ymask = 0x00007f00;
    yshift = 8;
    xshift = 1;
    polshift = 0;
    polmask = 0x00000001;

    retinalSize = 128;

    wrapAdd = 0;
    //fopen_s(&fp,"events.txt", "w"); //Use the unmasked_buffer
    uEvents = fopen("./uevents.txt","w");
}

unmask::~unmask() {
}

list<AER_struct> unmask::unmaskData(char* i_buffer, int i_sz) {
//    cout << "Size of the received packet to unmask : " << i_sz << endl;
    AER_struct sAER;
    list<AER_struct> l_AER;
    for (int j=0; j<i_sz; j+=4) {
        if((i_buffer[j+3]&0x80)==0x80) {
            // timestamp bit 15 is one -> wrap
            // now we need to increment the wrapAdd

            wrapAdd+=0x4000/*L*/; //uses only 14 bit timestamps
            //System.out.println("received wrap event, index:" + eventCounter + " wrapAdd: "+ wrapAdd);
            //NumberOfWrapEvents++;
        }
        else if  ((i_buffer[j+3]&0x40)==0x40  ) {
            // timestamp bit 14 is one -> wrapAdd reset
            // this firmware version uses reset events to reset timestamps
            //write(file_desc,reset,1);//this.resetTimestamps();
            //buffer_msg[0] = 6;
            //write(file_desc,buffer_msg,1);
            wrapAdd=0;
            // log.info("got reset event, timestamp " + (0xffff&((short)aeBuffer[i]&0xff | ((short)aeBuffer[i+1]&0xff)<<8)));
        }
        else {
            //unmask the data
            unsigned int part_1 = 0x00FF&i_buffer[j];
            unsigned int part_2 = 0x00FF&i_buffer[j+1];
            unsigned int part_3 = 0x00FF&i_buffer[j+2];
            unsigned int part_4 = 0x00FF&i_buffer[j+3];
            unsigned int blob = (part_1)|(part_2<<8);
            unmaskEvent(blob, cartX, cartY, polarity);
            timestamp = ((part_3)|(part_4<<8))/*&0x7fff*/;
            timestamp+=wrapAdd;

            sAER.x = cartX;
            sAER.y = cartY;
            sAER.pol = polarity;
            sAER.ts = timestamp;
            l_AER.push_back(sAER);
            //fprintf(uEvents,"%d\t%d\t%d\t%u\n", cartX, cartY, polarity, timestamp);
        }
    }
    sAER.x = -1.0;
    sAER.y = -1.0;
    sAER.pol = -1.0;
    sAER.ts = 0;
    l_AER.push_back(sAER);
    fprintf(uEvents,"%d\t%d\t%d\t%u\n", -1, -1, -1, -1);
    return l_AER;
}

void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol) {
    x = (short)(retinalSize-1) - (short)((evPU & xmask)>>xshift);
    y = (short) ((evPU & ymask)>>yshift);
    pol = ((short)((evPU & polmask)>>polshift)==0)?-1:1;	//+1 ON, -1 OFF
}
