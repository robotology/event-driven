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




#include "VelocityBuffer.h"

VelocityBuffer::VelocityBuffer(){
    vxMin = DBL_MAX;
    vxMax = -1;
    vyMin = DBL_MAX;
    vyMax = -1;
    size = 0;
}

bool VelocityBuffer::addData(short x, short y, double vx, double vy, unsigned long ts, double reliablity){
    double tmp;
    if (size >= BUFFER_LENGTH)
        return false;

    // check Min/Max velocity in X direction
    tmp = fabs(vx);
    if (tmp < vxMin)
       vxMin = tmp;
    if (tmp > vxMax)
       vxMax = tmp;
    // check Min/Max velocity in Y direction
    tmp = fabs(vy);
    if (tmp < vyMin)
       vyMin = tmp;
    if (tmp > vyMax)
       vyMax = tmp;

    Xs[size] = x;
    Ys[size] = y;
    Vxs[size] = vx;
    Vys[size] = vy;
    TSs[size] = ts;
    size++;
    return true;
}

VelocityBuffer::~VelocityBuffer(){}

bool VelocityBuffer::addDataCheckFull(short x, short y, double vx, double vy, unsigned long ts, double reliablity){
    double tmp;
    Xs[size] = x;
    Ys[size] = y;
    Vxs[size] = vx;
    Vys[size] = vy;
    TSs[size] = ts;
    size++;

    //std::cout << x << " " << y << " " << vx << " " << vy << std::endl;

    // check Min/Max velocity in X direction
    tmp = fabs(vx);
    if (tmp < vxMin)
        vxMin = tmp;
    if (tmp > vxMax)
        vxMax = tmp;
    // check Min/Max velocity in Y direction
    tmp = fabs(vy);
    if (tmp < vyMin)
        vyMin = tmp;
    if (tmp > vyMax)
        vyMax = tmp;

    if (size == BUFFER_LENGTH)
        return true;

    return false;
}

bool VelocityBuffer::isFull(){
    return size == BUFFER_LENGTH;
}

bool VelocityBuffer::isEmpty(){
    return size == 0;
}


void VelocityBuffer::emptyBuffer(){
    size = 0;
    vxMin = DBL_MAX;
    vxMax = -1;
    vyMin = DBL_MAX;
    vyMax = -1;
}

bool VelocityBuffer::read(ConnectionReader & connection){
    connection.convertTextMode(); // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag!=BOTTLE_TAG_LIST+ BOTTLE_TAG_BLOB+ BOTTLE_TAG_DOUBLE)
        return false;
    size = connection.expectInt();
    for (int i = 0; i < size; ++i) {
        Xs[i] = connection.expectInt();
    }
    for (int i = 0; i < size; ++i) {
        Ys[i] = connection.expectInt();
    }
    for (int i = 0; i < size; ++i) {
       Vxs[i] = connection.expectDouble();
    }
    for (int i = 0; i < size; ++i) {
       Vys[i] = connection.expectDouble();
    }
    for (int i = 0; i < size; ++i) {
	   TSs[i] = (unsigned long) connection.expectInt();
	}




    vxMin = connection.expectDouble();
    vxMax = connection.expectDouble();
    vyMin = connection.expectDouble();
    vyMax = connection.expectDouble();

    return true;
}

bool VelocityBuffer::write(ConnectionWriter & connection){
    connection.appendInt(BOTTLE_TAG_LIST+ BOTTLE_TAG_BLOB+ BOTTLE_TAG_DOUBLE);
    connection.appendInt(size);
    for (int i = 0; i < size; ++i) {
        connection.appendInt(Xs[i]);
    }
    for (int i = 0; i < size; ++i) {
        connection.appendInt(Ys[i]);
    }
    for (int i = 0; i < size; ++i) {
        connection.appendDouble(Vxs[i]);
    }
    for (int i = 0; i < size; ++i) {
        connection.appendDouble(Vys[i]);
    }
    for (int i = 0; i < size; ++i) {
		connection.appendInt((int) TSs[i]);
	}


    connection.appendDouble(vxMin);
    connection.appendDouble(vxMax);
    connection.appendDouble(vyMin);
    connection.appendDouble(vyMax);
    connection.convertTextMode(); // if connection is text-mode, convert!
    return true;
}

void VelocityBuffer::setData(const VelocityBuffer & src){
    size = src.size;
    for (int i = 0; i < size; ++i) {
        Xs[i]= src.Xs[i];
        Ys[i]=src.Ys[i];
        Vxs[i]=src.Vxs[i];
        Vys[i]=src.Vys[i];
        TSs[i] = src.TSs[i];
    }

    vxMin = src.vxMin;
    vxMax = src.vxMax;
    vyMin = src.vyMin;
    vyMax = src.vyMax;
}

void VelocityBuffer::setVx(int idx, double vx ){
    double tmp;
    Vxs[idx] = vx;

    tmp = fabs(vx);
    if (tmp < vxMin)
        vxMin = tmp;
    if (tmp > vxMax)
        vxMax = tmp;
}

void VelocityBuffer::setVy(int idx, double vy ){
    double tmp;
    Vys[idx] = vy;
    tmp = fabs(vy);
    if (tmp < vyMin)
        vyMin = tmp;
    if (tmp > vyMax)
        vyMax = tmp;
}

double VelocityBuffer::getVxMin(){
    return vxMin;
}

double VelocityBuffer::getVxMax(){
    return vxMax;
}

double VelocityBuffer::getVyMin(){
    return vyMin;
}
double VelocityBuffer::getVyMax(){
    return vyMax;
}
