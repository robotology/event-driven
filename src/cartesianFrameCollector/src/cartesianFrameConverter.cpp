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
 * @file cartesianFrameConverter.cpp
 * @brief A class inherited from the bufferefPort (see header cartesianFrameConverter.h)
 */

#include <iCub/cartesianFrameConverter.h>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

cFrameConverter::cFrameConverter():convert_events(128,128) {
    retinalSize=128;
    outputWidth=320;
    outputHeight=240;
    unmask_events.start();
}

cFrameConverter::~cFrameConverter() {
    delete &unmask_events;
    delete &convert_events;
}

void cFrameConverter::onRead(sendingBuffer& i_ub) {
    //cout << "C_yarpViewer::onRead(unmaskedbuffer& i_ub)" << endl;
    //start_u = clock();
    unmask_events.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket());
    //start_p = clock();
    //stop = clock();
    /*
    cout << "Unmask task : " << (stop/CLOCKS_PER_SEC) - (start_u/CLOCKS_PER_SEC) << endl
        << "Printing task : " << (stop/CLOCKS_PER_SEC) - (start_p/CLOCKS_PER_SEC) << endl;
    */
}

void cFrameConverter::getMonoImage(ImageOf<PixelMono>* image, unsigned int minCount, unsigned int maxCount){
    assert(image!=0);
    image->resize(outputWidth,outputHeight);
    unsigned char* pImage=image->getRawImage();
    int imagePadding=image->getPadding();
    int imageRowSize=image->getRowSize();
    int* pBuffer = unmask_events.getEventBuffer();
    unsigned int* pTime   = unmask_events.getTimeBuffer();
    double a = 1,b = 0;
    int maxValue=unmask_events.getMaxValue();
    int minValue=unmask_events.getMinValue();
    
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value= *pBuffer++;
            unsigned int timestamp = *pTime++;
            if((timestamp > minCount) && (timestamp < maxCount)) {
                //value=a * value + b;
                *pImage++ = (unsigned char) 127 + value ;
            }
        }
        pImage+=imagePadding;
    }
}

void cFrameConverter::clearMonoImage() {
    unmask_events.cleanEventBuffer();
}

