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

/// \defgroup emorphLib emorphLib
/// \defgroup vBottle vBottle
/// \ingroup emorphLib
/// \brief yarp::os::Bottle wrapper for sending events through the yarp system with
/// particular attention to using data dumper and data players

#ifndef __vBottle__
#define __vBottle__


#include <yarp/os/Bottle.h>
#include <iCub/emorph/vCodec.h>
#include <iCub/emorph/vQueue.h>

namespace emorph {

class vBottle : public yarp::os::Bottle {

public:

    //constructors shouldn't change from Bottle
    vBottle() : yarp::os::Bottle() {}

    //you can only modify contents by adding events and append other vBottles
    void addEvent(emorph::vEvent &e);

    void append(vBottle &eb)
    {
        append<emorph::vEvent>(eb);
    }

    template<class T> void append(vBottle &eb)
    {
        //we need to access the data in eb as if it were a normal bottle
        //so we cast it to a Bottle

        //TODO: just make sure the functions are available but protected should
        //      make the casting unnecessary
        yarp::os::Bottle * bb = dynamic_cast<yarp::os::Bottle *>(&eb);

        //for each list of events
        for(int tagi = 0; tagi < bb->size(); tagi+=2) {

            //get the appended event type
            const std::string tagname = bb->get(tagi).asString();
            if(!tagname.size()) {
                std::cerr << "Warning: Could not get tagname during vBottle append."
                             "Check vBottle integrity." << std::endl;
                continue;
            }

            //check to see if we want to append this event type
            vEvent * e = emorph::createEvent(tagname);
            if(!e) {
                std::cerr << "Warning: could not get bottle type during vBottle::"
                             "append<>(). Check vBottle integrity." << std::endl;
                continue;
            }
            if(!dynamic_cast<T*>(e)) continue;


            //we want to append these events so get the data from bb
            yarp::os::Bottle *b_from = bb->get(tagi+1).asList();
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
            vEvent * e = emorph::createEvent(Bottle::get(i).asString());
            if(!e) {
                std::cerr << "Warning: could not get bottle type during vBottle::"
                             "get<>(). Check vBottle integrity." << std::endl;
                continue;
            }

            //and if e is of type T we can continue to get the events
            if(!dynamic_cast<T*>(e)) {
                delete(e);
                continue;
            }

            //we get the (EVENTS)
            Bottle * b = Bottle::get(i+1).asList();
            if(!b) {
                std::cerr << "Warning: could not get event data as a list after "
                             "getting correct tag (e.g. AE) in vBottle::getAll(). "
                             "Check vBottle integrity" << std::endl;
                delete(e);
                break;
            }

            //and decode each one also creating the memory with clone
            int pos = 0;
            while(pos < b->size()) {
                if(e->decode(*b, pos)) {
                    q.push_back(e->clone());
                }
            }

            //finally we don't need the last event
            delete(e);

        }

    }

    template<class T> vQueue getSorted()
    {
        vQueue q = get<T>();
        q.sort();
        return q;
    }

    vQueue getAll() {
        vQueue q = this->get<vEvent>(); //all events are of type vEvent
        return q;
    }

    vQueue getAllSorted() {
        vQueue q = getAll();
        q.sort(true);
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
    void findGroup();
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

} //end namespace emorph

#endif /*__vBottle__*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------
