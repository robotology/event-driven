// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Francesco Rea
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <eventBottle.h>

using namespace std;
using namespace yarp::os;

eventBottle::eventBottle() {
    printf("default Constructor \n");
    packet = new Bottle();
    size_of_the_packet=0;
    packetPointer = new char[32000];
    //packetPointer = 0;
}

eventBottle::eventBottle(char* i_data, int i_size) {
    packet = new Bottle();
    packet->clear();
    int word;
    
    for(int i = 0 ; i < i_size ;) {
        word = 0;
        for (int j = 0 ; j < 4 ; j++){
            //printf(" %01x,",(unsigned char) *i_data);
            //word = word << (8 * j);
            int value = (unsigned char) *i_data << (8 * j);
            word = word | value;
            i_data++;
            i++;
        }
        //printf("adding word %08x \n", word);
        packet->addInt(word);
    }
    
    size_of_the_packet = packet->size();
    packetPointer = new char;   // never used 
       
    /*
    packetPointer = new char[32000];
    memcpy(packetPointer, i_data, i_size);
    size_of_the_packet = i_size;
    */    
}

eventBottle::eventBottle(const eventBottle& buffer) {
    
    packet = new Bottle();
    if (buffer.size_of_the_packet > 0) {
        //memcpy(packet, buffer.packet, sizeof(char) * buffer.size_of_the_packet);
        packet =  buffer.packet;
    }
    size_of_the_packet = buffer.size_of_the_packet;
    packetPointer = new char;
    
    /*
    packetPointer = new char[32000];
    if (buffer.size_of_the_packet > 0)
        memcpy(packetPointer, buffer.packetPointer, sizeof(char) * buffer.size_of_the_packet);
    size_of_the_packet = buffer.size_of_the_packet;
    */
}

eventBottle::~eventBottle() {
    delete packetPointer;
    delete packet;
}

void eventBottle::operator=( eventBottle& buffer) {
    if (buffer.size_of_the_packet > 0) {
        Bottle* b = buffer.get_packet();
        //int v = b->pop().asInt();
        //printf("extracted value %d \n",v);
        packet->copy(*b);
        //packet = buffer.packet;
    }
    size_of_the_packet = buffer.size_of_the_packet;
    //size_of_the_packet = 4;
    //packetPointer      = buffer.packetPointer;
    
    /*
    if (buffer.size_of_the_packet > 0)
        memcpy(packetPointer, buffer.packetPointer, sizeof(char) * 4 * buffer.size_of_the_packet);
    size_of_the_packet = buffer.size_of_the_packet;
    */
}

void eventBottle::set_data(char* i_data, int i_size) {
    //memcpy(packet, i_data, i_size);
    //size_of_the_packet = i_size;
    unsigned long word;
    for(int i = 0 ; i < i_size ;) {
        word = 0;
        for (int j = 0 ; j < 4 ; j++){
            word = word << 4;
            word &= *i_data;
            i++;
        }
        packet->addInt(word);
    }
    size_of_the_packet = packet->size();
    packetPointer = new char;
    
    /*
    memcpy(packetPointer, i_data, i_size);
    size_of_the_packet = i_size;
    */
}


bool eventBottle::write(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
    connection.appendInt(2);        // four elements
    
    //bool nullB = packet->isNull();
    //printf("packet is null %d \n", nullB);
    //ConstString strBottle = packet->toString();
    size_t binaryDim;
    packetPointer = (char*) packet->toBinary(&binaryDim);
    //connection.appendInt(size_of_the_packet);
    connection.appendInt(binaryDim);
    int ceilSizeOfPacket = size_of_the_packet * 4;   // number of 32bit word times 4bytes
    printf("size of the packet %d \n", ceilSizeOfPacket);
    printf("comparing with the binaryDim: %d \n", binaryDim);
    //printf("dimension of the bottle in bytes %d \n", binaryDim);
    //connection.appendBlock(packetPointer,ceilSizeOfPacket); //casting bottle into char*
    connection.appendBlock(packetPointer,binaryDim);
    connection.convertTextMode();   // if connection is text-mode, convert!
    return true;
}


/*
bool eventBottle::write(yarp::os::ConnectionWriter& connection) {
    printf("writing down the bottle \n");
    packet->write(connection);
}
*/


bool eventBottle::read(yarp::os::ConnectionReader& connection) {
    
    connection.convertTextMode();   // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag != BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
       return false;
    int ct = connection.expectInt();
    if (ct!=2)
        return false;
    size_of_the_packet = connection.expectInt();  //corresponds to binaryDim in write
    //int ceilSizeOfPacket = size_of_the_packet * 4; // number of 32 bit word times 4bytes
  
    connection.expectBlock(packetPointer, size_of_the_packet);
    packet->fromBinary(packetPointer,size_of_the_packet);
    
    return true;
}


/*
bool eventBottle::read(yarp::os::ConnectionReader& connection) {
    packet->read(connection);
}
*/
