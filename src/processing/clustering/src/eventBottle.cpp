// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Francesco Rea
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "eventBottle.h"
#define wordDimension 4  //bytes

using namespace std;
using namespace yarp::os;

eventBottle::eventBottle() {
    //printf("default Constructor \n");
    packet = new Bottle;
    size_of_the_packet  = 0;
    size_of_the_bottle  = 0;
    bytes_of_the_packet = 0;
    packetPointer = new char[320000];
    fout = fopen("cartesianFrameCollector.eventBottleHandler.txt", "w+");
}

eventBottle::eventBottle(char* i_data, int i_size) {
    //printf("parametric constructor \n");
    packet = new Bottle();
    packet->clear();


    // copying the data
    int word;
    for(int i = 0 ; i < i_size ;) {
        word = 0;
        for (int j = 0 ; j < wordDimension ; j++){
            int value = (unsigned char) *i_data << (8 * j);
            word = word | value;
            i_data++;
            i++;
        }
        packet->addInt(word);
    }
    size_of_the_bottle = packet->size();
    packetPointer = 0;

    fout = fopen("cartesianFrameCollector.eventBottleHandler.txt", "w+");
}

/**
 * constructor from a bottle
 */
eventBottle::eventBottle(Bottle* p) {
    packet = new Bottle();

    size_of_the_bottle = p->size();
    size_of_the_packet = size_of_the_bottle;
    bytes_of_the_packet = size_of_the_packet * 4;
    int word;

    for(int i = 0 ; i < size_of_the_bottle ;i++) {
        word = p->get(i).asInt();
        packet->addInt(word);
    }

    packetPointer = 0;
    fout = fopen("eventBottle.debug.txt", "w+");
}

/**
 *@brief copy constructor
 */
eventBottle::eventBottle(const eventBottle& buffer) {
    packet = new Bottle();
    if (buffer.size_of_the_bottle > 0) {
        packet = buffer.packet;
    }
    size_of_the_packet = buffer.size_of_the_packet;
    bytes_of_the_packet = buffer.bytes_of_the_packet;
    size_of_the_bottle = buffer.size_of_the_bottle;
    packetPointer = 0;

    fout = fopen("cartesianFrameCollector.eventBottleHandler.txt", "w+");
}

eventBottle::~eventBottle() {
    fclose(fout);
    delete[] packetPointer;
    delete packet;
}

void eventBottle::operator=( eventBottle& buffer) {

    if (buffer.size_of_the_bottle > 0) {
        Bottle* b = buffer.get_packet();
        //packet->copy(*b); <------alternative B works as a copy of the pointer which should be avoided

        delete packet;
        packet = new Bottle();

        for (int i = 0; i < buffer.size_of_the_bottle; i++) {
            int value = b->get(i).asInt();
            packet->addInt(value);
        }

    }
    size_of_the_packet  = buffer.size_of_the_packet;
    bytes_of_the_packet = buffer.bytes_of_the_packet;
    size_of_the_bottle  = buffer.size_of_the_bottle;
}

void eventBottle::set_data(char* i_data, int i_size) {
    unsigned long word;
    for(int i = 0 ; i < i_size ;) {
        word = 0;
        for (int j = 0 ; j < wordDimension ; j++){
            word = word << 4;
            word &= *i_data;
            i++;
        }
        packet->addInt(word);
    }
    size_of_the_bottle = packet->size();
    packetPointer = 0;
}

void eventBottle::set_data(Bottle* p) {
    packet = p;
    size_of_the_bottle = packet->size();
    packetPointer = 0;
}


bool eventBottle::write(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
    connection.appendInt(2);        // four elements

    size_t binaryDim;
    size_of_the_bottle  = packet->size();
    size_of_the_packet  = size_of_the_bottle;
    bytes_of_the_packet = size_of_the_packet * 4;
    //packetPointer = (char*) packet->toBinary(&binaryDim);
    //bytes_of_the_packet = binaryDim;
    //size_of_the_packet  = bytes_of_the_packet >> 2;
    connection.appendInt(bytes_of_the_packet);
    //connection.appendInt(size_of_the_packet);
    //bytes_of_the_packet = size_of_the_packet * wordDimension;   // number of 32bit word times 4bytes

    //--------------------------------------------------------------------------------------------
    // -------- serialisation of the bottle ---------------------------
    unsigned char tmpChar;
    char *p = packetPointer;

    for(int i = 0 ; i < size_of_the_packet ; i++) {
        int value = packet->get(i).asInt();

        for (int j = 0 ; j < wordDimension ; j++){
            int tmpInt   = (value & 0x000000FF) ;
            tmpChar      =  (unsigned char) tmpInt;
            //printf("%02x %02x ",tmpInt,tmpChar);
            value = value >> 8;
            *p = tmpChar;
            p++;
        }
    }
    //----------------------------------------------------------------------------------------------


#if VERBOSE
    int word;
    printf("packet %d %d \n %s \n", size_of_the_packet, binaryDim, packet->toString().c_str());
    char* i_data = packetPointer;
    for(int i = 0 ; i < binaryDim ;) {
        word = 0;
        for (int j = 0 ; j < wordDimension ; j++){
            int value = (char) *i_data << (8 * j);
            word = word | value;
            i_data++;
            i++;
        }
        printf(">%08X ", word);
    }
    printf("\n");
#endif

    //-----------------------------------------------------------------------------------------------
    connection.appendBlock(packetPointer,bytes_of_the_packet);   //serializing bottle into char*
    //printf("packet \n %s \n", packet->toString().c_str());
    connection.convertTextMode();   // if connection is text-mode, convert!
    return true;
}


bool eventBottle::read(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();   // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag != BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT) {
       return false;
    }
    int ct = connection.expectInt();
    if (ct!=2) {
        return false;
    }
    bytes_of_the_packet = connection.expectInt();
    size_of_the_packet = bytes_of_the_packet / wordDimension;      // number of 32 bit word times 4bytes
    connection.expectBlock(packetPointer,bytes_of_the_packet);

    //printf(" eventBottle::read:size_of_the_packet %d bytes_of_the_packet %d \n", size_of_the_packet, bytes_of_the_packet);


    // ---------------------------------------------------------------------------------------------------------
    // ------------------------ deserialisation of the bottle -------------------------------------
    //printf("bytes of the packet %d \n",bytes_of_the_packet );
    int word;
    char* i_data  = packetPointer;
    packet->clear(); // clear the bottle before any other adding

    unsigned char tmpChar;
    for(int i = 0 ; i < bytes_of_the_packet;) {
        word = 0;
        for (int j = 0 ; j < 4 ; j++){
            tmpChar = (unsigned char) *i_data;
            //printf("%02x ", *i_data );
            int value =  tmpChar << (8 * j);
            word = word | value;
            i_data++;
            i++;
        }
        packet->addInt(word);
        //printf("= %08x \n", word);

    }

    //----------------------------------------------------------------------------------------------------------

    //packet->fromBinary(packetPointer,bytes_of_the_packet);
    size_of_the_bottle = packet->size();
    return true;
}


