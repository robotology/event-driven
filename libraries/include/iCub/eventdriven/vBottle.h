/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

/// \defgroup Library Library
/// \defgroup vBottle vBottle
/// \ingroup Library
/// \brief yarp::os::Bottle wrapper for sending events through the yarp system with
/// particular attention to using data dumper and data players

#ifndef __VBOTTLE__
#define __VBOTTLE__

#include <yarp/os/Bottle.h>
#include "iCub/eventdriven/vCodec.h"

namespace ev {

class vBottle : public yarp::os::Bottle {

public:

    //constructors shouldn't change from Bottle
    vBottle() : yarp::os::Bottle() {}

    //you can only modify contents by adding events and append other vBottles
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

    void append(vBottle &eb)
    {
        append<vEvent>(eb);
    }

    template<class T> void append(vBottle &eb)
    {

        //for each list of events
        for(int tagi = 0; tagi < eb.yarp::os::Bottle::size(); tagi+=2) {

            //get the appended event type
            const std::string tagname = eb.yarp::os::Bottle::get(tagi).asString();
            if(!tagname.size()) {
                std::cerr << "Warning: Could not get tagname during vBottle append."
                             "Check vBottle integrity." << std::endl;
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
    template<class T> vQueue get() {

        vQueue q;
        addtoendof<T>(q);
        return q;

    }

    template<class T> void addtoendof(vQueue &q) {

        //the bottle is stored as TAG (EVENTS) TAG (EVENTS)
        for(int i = 0; i < Bottle::size(); i+=2) {

            //so for each TAG we create an event of that type
            event<> e = createEvent(Bottle::get(i).asString());
            if(!e) {
                std::cerr << "Warning: could not get bottle type during vBottle::"
                             "get<>(). Check vBottle integrity." << std::endl;
                continue;
            }

            //and if e is of type T we can continue to get the events
            if(!std::dynamic_pointer_cast<T>(e)) {
                continue;
            }

            //we get the (EVENTS)
            Bottle * b = Bottle::get(i+1).asList();
            if(!b) {
                std::cerr << "Warning: could not get event data as a list after "
                             "getting correct tag (e.g. AE) in vBottle::getAll(). "
                             "Check vBottle integrity" << std::endl;
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

    template<class T> vQueue getSorted()
    {
        vQueue q = get<T>();
        qsort(q);
        return q;
    }

    vQueue getAll() {
        vQueue q = this->get<vEvent>(); //all events are of type vEvent
        return q;
    }

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


private:

    //you cannot use any of the following functions
    void add();
    void addDict();
    void addDouble();
    void addInt();
    void addList();
    void addString();
    void addVocab();
    void fromString(const yarp::os::ConstString& text);
    void append(const yarp::os::Bottle &alt);
    void copy();
    void copyPortable();
    //yarp::os::Value& find(const yarp::os::ConstString &key);
    //yarp::os::Value& find(const ConstString &key) : Bottle::find(const yarp::os::ConstString &key) {};
    //Bottle& findGroup(const yarp::os::ConstString& key) const;
    //void findGroup();
    using yarp::os::Bottle::findGroup;
    using yarp::os::Bottle::find;

    yarp::os::Bottle tail() const;


    int getInt();
    yarp::os::ConstString getString();
    double getDouble();
    Bottle* getList();
    yarp::os::Value& get();

    yarp::os::Value pop();

};

class vBottleMimic : public yarp::os::Portable {

private:

    std::vector<YARP_INT32> header1;
    std::vector<char> header2;
    std::vector<YARP_INT32> header3;

    const char * datablock;
    unsigned int datalength;
    const static unsigned int MINELSZ = sizeof(YARP_INT32) * 2;

public:

    vBottleMimic() {
        header1.push_back(BOTTLE_TAG_LIST); //bottle code
        header1.push_back(2); //elements in bottle "AE" then bottle data
        header1.push_back(BOTTLE_TAG_STRING); //code for string
        header1.push_back(2); // length of string
        header2.push_back('A');
        header2.push_back('E');
        header3.push_back(BOTTLE_TAG_LIST|BOTTLE_TAG_INT); // bottle code + specialisation with ints
        header3.push_back(0); // <- set the number of ints here (2 * #v's)
    }

    void setdata(const char * datablock, unsigned int datalength) {
        header3[1] = 2 * (datalength / MINELSZ); //forced to be x8
        this->datablock = datablock;
        this->datalength = MINELSZ * header3[1] / 2; //forced to be x8

    }

    virtual bool read(yarp::os::ConnectionReader& connection) {
                return false;
    }

    virtual bool write(yarp::os::ConnectionWriter& connection) {

        connection.appendBlock((const char *)header1.data(),
                                       header1.size() * sizeof(YARP_INT32));
        connection.appendBlock((const char *)header2.data(),
                                       header2.size() * sizeof(char));
        connection.appendBlock((const char *)header3.data(),
                                       header3.size() * sizeof(YARP_INT32));
        connection.appendBlock(datablock, datalength);

        return !connection.isError();
    }

};

} //end namespace ev

#endif /*__vBottle__*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------
