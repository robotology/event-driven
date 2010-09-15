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

#include <iCub/cartesian_frame_converter.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

cframeConverter::cframeConverter():convert_events(128,128) {
}

cframeConverter::~cframeConverter() {
}

void cframeConverter::onRead(sendingBuffer& i_ub) {
    cout << "C_yarpViewer::onRead(unmaskedbuffer& i_ub)" << endl;
    start_u = clock();
    list<AER_struct> datas = unmask_events.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket());
    start_p = clock();
    ImageOf<PixelMono16> frame = convert_events.create_frame(datas);
    convert_events.send_frame(frame);
    stop = clock();
    cout << "Unmask task : " << (stop/CLOCKS_PER_SEC) - (start_u/CLOCKS_PER_SEC) << endl
        << "Printing task : " << (stop/CLOCKS_PER_SEC) - (start_p/CLOCKS_PER_SEC) << endl;
}
