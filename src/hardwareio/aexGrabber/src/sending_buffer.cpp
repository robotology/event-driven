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
 * @file sending_buffer.h
 * @brief Sends information on a YARP port (see header sending_buffer.h)
 */

#include <sending_buffer.h>
#include <yarp/os/Bottle.h>

using namespace std;
using namespace yarp::os;

sendingBuffer::sendingBuffer() {
    packet = new char[SIZE_OF_DATA];
    size_of_the_packet=0;
}

sendingBuffer::sendingBuffer(char* i_data, int i_size) {
    packet = new char[SIZE_OF_DATA];
    memcpy(packet, i_data, SIZE_OF_DATA);
    size_of_the_packet = i_size;
}

sendingBuffer::~sendingBuffer() {
    delete[] packet;
}

void sendingBuffer::set_data(char* i_data, int i_size) {
    memcpy(packet, i_data, SIZE_OF_DATA);
    size_of_the_packet = i_size;
}

bool sendingBuffer::write(ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT);
    connection.appendInt(2); // four elements
    connection.appendInt(size_of_the_packet);
    connection.appendBlock (packet, SIZE_OF_DATA);
    connection.convertTextMode(); // if connection is text-mode, convert!
    return true;
}

bool sendingBuffer::read(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode(); // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag!=BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
        return false;
    int ct = connection.expectInt();
    if (ct!=2)
        return false;
    size_of_the_packet = connection.expectInt();
    connection.expectBlock(packet, SIZE_OF_DATA);
    return true;
}

