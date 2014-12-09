/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Arren Glover (@itt.it)
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
 * \defgroup
 * @ingroup emorph_lib
 *
 * Data transport method for the eMorph project
 *
 * Author: Arren Glover 2014
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef __eBottle__
#define __eBottle__


#include <yarp/os/all.h>
#include <iCub/emorph/eCodec.h>

namespace emorph {

class eBottle : public yarp::os::Bottle {

public:

    //constructors shouldn't change from Bottle
    eBottle() : yarp::os::Bottle() {}

    //you can only modify contents by adding events and append other eBottles
    void addEvent(emorph::eEvent &e);\
    void append(eBottle &eb);

    void getAll(eEventQueue &q);
    void getAllSorted(eEventQueue &q);

    //you can then get events from the eBottle
    //must be defined in header for template to work
    template<class T> int get(emorph::eEventQueue &q)
    {
        int r = 0;
        T t;

        Bottle * b = find(t.getType()).asList();

        int pos = 0;
        while(pos < b->size()) {
            q.push_back(t.decode(*b, pos));
            r++;
        }

        return r;

    }

    template<class T> int getSorted(emorph::eEventQueue &q)
    {
        int r = get<T>(q);
        q.sort();
        return r;
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

} //end namespace emorph

#endif /*__eBottle__*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------
