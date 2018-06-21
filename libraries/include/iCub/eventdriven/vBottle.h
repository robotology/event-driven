/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VBOTTLE__
#define __VBOTTLE__

#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include "iCub/eventdriven/vCodec.h"
#include <iostream>

namespace ev {

/// \brief yarp::os::Bottle wrapper for sending events through the yarp system
/// with ensuring compatibility with yarpdatadumper and yarpdataplayer
class vBottle : public yarp::os::Bottle {

public:

    //constructors shouldn't change from Bottle
    /// \brief default constructor
    vBottle() : yarp::os::Bottle() {}

    //you can only modify contents by adding events and append other vBottles
    /// \brief add a single event to the vBottle
    void addEvent(event<> e) {

        //first append a searchable string
        //yarp::os::Bottle::addString(e.getType());
        yarp::os::Bottle * b = yarp::os::Bottle::find(e->getType()).asList();

        if(!b) {
            yarp::os::Bottle::addString(e->getType());
            b = &(yarp::os::Bottle::addList());
        }

        //add the coded event to the end of the bottle
        e->encode(*b);
        //b->append(e.encode());

    }
    /// \brief add all the contents of a vBottle to the current vBottle
    void append(vBottle &eb)
    {
        append<vEvent>(eb);
    }

    /// \brief append
    template<class T> void append(vBottle &eb)
    {

        //for each list of events
        for(int tagi = 0; tagi < eb.yarp::os::Bottle::size(); tagi+=2) {

            //get the appended event type
            const std::string tagname =
                    eb.yarp::os::Bottle::get(tagi).asString();
            if(!tagname.size()) {
                yError() << "Could not get tagname during vBottle append."
                             "Check vBottle integrity.";
                continue;
            }

            //check to see if we want to append this event type
            event<> e = createEvent(tagname);
            if(!e) {
                std::cerr << "Warning: could not get bottle type during vBottle::"
                             "append<>(). Check vBottle integrity." << std::endl;
                continue;
            }
            if(!std::dynamic_pointer_cast<T>(e)) continue;


            //we want to append these events so get the data from bb
            yarp::os::Bottle *b_from = eb.yarp::os::Bottle::get(tagi+1).asList();
            if(!b_from->size()) {
                std::cerr << "Warning: From-list empty during vBottle append."
                             "Check vBottle integrity." << std::endl;
                continue;
            }

            //get the correct bottle to append to (or create a new one)
            yarp::os::Bottle *b_to = yarp::os::Bottle::find(tagname).asList();
            if(!b_to) {
                yarp::os::Bottle::addString(tagname);
                b_to = &(yarp::os::Bottle::addList());
            }

            //and do it
            b_to->append(*b_from);
        }

    }

    //all get functions call this to do the meat of the getting function
    /// \brief decode a single event-type into a vQueue from the vBottle
    template<class T> vQueue get() {

        vQueue q;
        addtoendof<T>(q);
        return q;

    }

    /// \brief add a specific event-type from the vBottle to the end of a
    ///  vQueue
    template<class T> void addtoendof(vQueue &q) {

        //the bottle is stored as TAG (EVENTS) TAG (EVENTS)
        for(int i = 0; i < Bottle::size(); i+=2) {

            //so for each TAG we create an event of that type
            event<> e = createEvent(Bottle::get(i).asString());
            if(!e) {
                yError() << "Warning: could not get bottle type during vBottle::"
                             "get<>(). Check vBottle integrity.";
                continue;
            }

            //and if e is of type T we can continue to get the events
            if(!std::dynamic_pointer_cast<T>(e)) {
                continue;
            }

            //we get the (EVENTS)
            Bottle * b = Bottle::get(i+1).asList();
            if(!b) {
                yError() << "Warning: could not get event data as a list after "
                             "getting correct tag (e.g. AE) in vBottle::getAll(). "
                             "Check vBottle integrity";
                break;
            }

            //and decode each one also creating the memory with clone
            //NOTE: push_back seems as fast as preallocation for a deque
            int pos_b = 0;
            while(pos_b < b->size()) {
                if(e->decode(*b, pos_b)) {
                    q.push_back(e->clone());
                }
            }
        }

    }

    /// \brief get a specific event-type and ensure they are in correct
    /// temporal order
    template<class T> vQueue getSorted()
    {
        vQueue q = get<T>();
        qsort(q);
        return q;
    }

    /// \brief decode all events into a vQueue
    vQueue getAll() {
        vQueue q = this->get<vEvent>(); //all events are of type vEvent
        return q;
    }

    /// \brief decode all events into a vQueue and ensure they are in correct
    /// temporal order
    vQueue getAllSorted() {
        vQueue q = getAll();
        qsort(q, true);
        return q;
    }

    //you can also access the following functions
    //not sure if some need to be blocked

//    Bottle::clear()
//    Bottle::check()
//    Bottle::fromBinary()
//    Bottle::fromString()
//    Bottle::getContext()
//    Bottle::getMonitor()
//    Bottle::getNullBottle()
//    Bottle::getReadType()
//    Bottle::getSpecialization();
//    Bottle::getType()
//    Bottle::getWriteType()
//    Bottle::hasChanged()
//    Bottle::isNull()
//    Bottle::onCommencement()
//    Bottle::onCompletion()
//    Bottle::operator !=()
//    Bottle::operator =()
//    Bottle::operator ==()
//    Bottle::setMonitor()
//    Bottle::setNested()
//    Bottle::size()
//    Bottle::specialize()
//    Bottle::Searchable
//    Bottle::toBinary()
//    Bottle::toString()

    using yarp::os::Bottle::find;

private:

    //you cannot use any of the following functions
    void add();
    void addDict();
    void addDouble();
    void addInt();
    void addList();
    void addString();
    void addVocab();
    void fromString(const std::string& text);
    void append(const yarp::os::Bottle &alt);
    void copy();
    void copyPortable();
    //yarp::os::Value& find(const yarp::os::ConstString &key);
    //yarp::os::Value& find(const ConstString &key) : Bottle::find(const yarp::os::ConstString &key) {};
    //Bottle& findGroup(const yarp::os::ConstString& key) const;
    //void findGroup();

    using yarp::os::Bottle::findGroup;

    yarp::os::Bottle tail() const;


    int getInt();
    std::string getString();
    double getDouble();
    Bottle* getList();
    yarp::os::Value& get();

    yarp::os::Value pop();

};

/// \brief a vBottle that avoids memory allocation where possible and can be
/// correctly decoded when read from a receiving port. Only works with a single
/// event-type.
class vBottleMimic : public yarp::os::Portable {

private:

    //headers
    std::vector<std::int32_t> header1;
    std::string header2;
    std::vector<std::int32_t> header3;

    //data
    const char * datablock;
    unsigned int datalength; //<- set the number of bytes here
    std::vector<std::int32_t> internaldata;

    //sizes
    unsigned int elementINTS;
    unsigned int elementBYTES;

public:

    /// \brief instantiate the correct headers for a Bottle
    vBottleMimic() {
        header1.push_back(BOTTLE_TAG_LIST); //bottle code
        header1.push_back(2); //elements in bottle "AE" then bottle data
        header1.push_back(BOTTLE_TAG_STRING); //code for string
        header1.push_back(2); // length of string
        header2 = "AE";
        header3.push_back(BOTTLE_TAG_LIST|BOTTLE_TAG_INT); // bottle code + specialisation with ints
        header3.push_back(0); // <- set the number of ints here (e.g. 2 * #v's)
        elementINTS = 2;
        elementBYTES = sizeof(std::int32_t) * elementINTS;
    }

    /// \brief for data already allocated in contiguous space. Just send this
    /// data on a port without memory reallocation.
    void setExternalData(const char * datablock, unsigned int datalength) {
        header3[1] = elementINTS * (datalength / elementBYTES); //forced to be x8
        this->datablock = datablock;
        this->datalength = elementBYTES * header3[1] / elementINTS; //forced to be x8

    }

    /// \brief send an entire vQueue. The queue is encoded and allocated into a single contiguous memory space. Faster than a standard vBottle.
    void setInternalData(const vQueue &q) {

        header3[1] = elementINTS * q.size(); //number of ints

        if((int)internaldata.size() < header3[1]) //increase internal mem if needed
            internaldata.resize(header3[1]);

        unsigned int pos = 0;
        for(unsigned int i = 0; i < q.size(); i++)  //decode the data into
            q[i]->encode(internaldata, pos);        //internal memeory

        if(pos != (unsigned int)header3[1])
            yError() << "vBottleMimic: encoding incorrect";

        this->datablock = (const char *)internaldata.data();
        this->datalength = elementBYTES * q.size();
    }

    /// \brief set the type of event that this vBottleMimic will send
    void setHeader(std::string eventtype) {
        header1[3] = eventtype.size();  //set the string length
        header2 = eventtype;            //set the string itself

        //get the elementsize THIS COULD BE MORE ELEGENT!!
        yarp::os::Bottle temp;
        event<> v = ev::createEvent(eventtype);
        v->encode(temp);
        elementINTS = temp.size();
        elementBYTES = sizeof(std::int32_t) * elementINTS;

    }

    /// \brief does nothing as this is a write-only port.
    virtual bool read(yarp::os::ConnectionReader& connection) {
                return false;
    }

    /// \brief write the data on the connection.
    virtual bool write(yarp::os::ConnectionWriter& connection) const {

        connection.appendBlock((const char *)header1.data(),
                                       header1.size() * sizeof(std::int32_t));
        connection.appendBlock(header2.c_str(), header1[3]);
        connection.appendBlock((const char *)header3.data(),
                                       header3.size() * sizeof(std::int32_t));
        connection.appendBlock(datablock, datalength);

        return !connection.isError();
    }

};

} //end namespace ev

#endif /*__vBottle__*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------
