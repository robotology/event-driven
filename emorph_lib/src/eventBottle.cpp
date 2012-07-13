// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Francesco Rea
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iCub/emorph/eventBottle.h>

using namespace std;
using namespace yarp::os;

eventBottle::eventBottle() {
    //printf("default Constructor \n");
    packet = new Bottle();
    size_of_the_packet=0;
    packetPointer = new char[320000];
    //packetPointer = 0;
}

eventBottle::eventBottle(char* i_data, int i_size) {
    //printf("parametric constructor \n");
    packet = new Bottle();
    packet->clear();
    int word;
    
    for(int i = 0 ; i < i_size ;) {
        word = 0;
        for (int j = 0 ; j < 4 ; j++){
            int value = (unsigned char) *i_data << (8 * j);
            word = word | value;
            i_data++;
            i++;
        }
        packet->addInt(word);
    }
    size_of_the_packet = packet->size();
    packetPointer = new char; 
       
}


eventBottle::eventBottle(Bottle* p) {
    packet = new Bottle();

    size_of_the_packet = p->size();    
    int word;
    
    for(int i = 0 ; i < size_of_the_packet ;i++) { 
        word = p->get(i).asInt();
        packet->addInt(word);
    }
          
    printf("packet size %d \n",packet->size() );      
    packetPointer = new char; 
}
  
eventBottle::eventBottle(const eventBottle& buffer) {
    
    packet = new Bottle();
    if (buffer.size_of_the_packet > 0) {
        packet =  buffer.packet;
    }
    size_of_the_packet = buffer.size_of_the_packet;
    packetPointer = new char;
    
}

eventBottle::~eventBottle() {
    delete[] packetPointer;
    delete packet;
}

void eventBottle::operator=( eventBottle& buffer) {
    
    if (buffer.size_of_the_packet > 0) {
        Bottle* b = buffer.get_packet();
        //packet->copy(*b); <------alternative B works as a copy of the pointer which should be avoid
        printf("in the copy operator = dim: %d  \n", buffer.size_of_the_packet);

        packet = new Bottle();

        for (int i = 0; i < buffer.size_of_the_packet; i++) {
            int value = b->get(i).asInt();
            //printf(" value  =  %08x \n", value);
            packet->addInt(value);
        }
           
    }
    size_of_the_packet = buffer.size_of_the_packet;
}

void eventBottle::set_data(char* i_data, int i_size) {
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
    
}

void eventBottle::set_data(Bottle* p) {
    packet = p;
    size_of_the_packet = packet->size();
    packetPointer = new char;
}


bool eventBottle::write(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
    connection.appendInt(2);        // four elements
    
    size_t binaryDim;
    packetPointer = (char*) packet->toBinary(&binaryDim);
    
    connection.appendInt(size_of_the_packet);
    int ceilSizeOfPacket = size_of_the_packet * 4;   // number of 32bit word times 4bytes

    //connection.appendBlock(packetPointer,ceilSizeOfPacket); //casting bottle into char*
    connection.appendBlock(packetPointer,binaryDim);
    connection.convertTextMode();   // if connection is text-mode, convert!
    return true;
}


bool eventBottle::read(yarp::os::ConnectionReader& connection) {
    
    connection.convertTextMode();   // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag != BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
       return false;
    int ct = connection.expectInt();
    if (ct!=2)
        return false;
    size_of_the_packet = connection.expectInt();  
    int binaryDim = size_of_the_packet * 4;      // number of 32 bit word times 4bytes
  
    connection.expectBlock(packetPointer,binaryDim );
    packet->fromBinary(packetPointer,binaryDim);
    
    return true;
}


