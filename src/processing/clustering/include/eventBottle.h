// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Francesco Rea
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef __eventBottle__
#define __eventBottle__


#include <yarp/os/all.h>
#include <cstring>


/**
 * portable class for the bottle of events
 */
class eventBottle : public yarp::os::Portable {
public:
    /**
     * default constructor
     */
    eventBottle();

    /**
     * @brief constructor
     * @param c pointer to the char buffer
     * @param i number of bytes
     */
    eventBottle(char* c, int i);

    /**
     * @brief constructor
     * @param b bottle that constructs the class
     */
    eventBottle(yarp::os::Bottle* b);

    /**
     * destructor
     */
    ~eventBottle();

    void operator =(eventBottle&);
    eventBottle(const eventBottle&);

    virtual bool write(yarp::os::ConnectionWriter&);
    virtual bool read (yarp::os::ConnectionReader&);

    /**
     * @brief function for setting the data afterward
     * @param c pointer to the char buffer of events
     * @param i number of bytes in the buffer
     */
    void set_data(char* c, int i);

    /**
     * @brief function for setting the data afterward
     * @param b bottle pointer
     */
    void set_data(yarp::os::Bottle* b);

    yarp::os::Bottle* get_packet()  { return packet; };
    int get_sizeOfPacket()  { return size_of_the_packet; };
    int get_bytesOfPacket() { return bytes_of_the_packet; };
    int get_sizeOfBottle()  { return size_of_the_bottle; };

private:
    yarp::os::Bottle* packet;
    char* packetPointer;
    int size_of_the_packet;      // dimension of the sent packet
    int bytes_of_the_packet;     // dimension in bytes of the sent packet
    int size_of_the_bottle;      // number of element that do belong to the bottle
    FILE* fout;
};


#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------
