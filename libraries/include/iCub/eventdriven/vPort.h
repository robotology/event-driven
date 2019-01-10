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

#ifndef __VGENPORT__
#define __VGENPORT__

#include <vector>
#include <yarp/os/all.h>
#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vtsHelper.h"

using namespace yarp::os;
using std::vector;
using std::string;
using std::int32_t;
using std::deque;

namespace ev {

class vPortableInterface : public Portable {

protected:

    //headers
    vector<int32_t> header1;
    string header2;
    vector<int32_t> header3;

    //data
    const char * datablock;
    unsigned int datalength; //<- set the number of bytes here
    vector<int32_t> internaldata;
    string event_type;
    unsigned int ints_to_read; //<- in integers

    //sizes
    unsigned int elementINTS;
    unsigned int elementBYTES;

public:

    /// \brief instantiate the correct headers for a Bottle
    vPortableInterface() {
        header1.push_back(BOTTLE_TAG_LIST); //bottle code
        header1.push_back(2); //elements in bottle "AE" then bottle data
        header1.push_back(BOTTLE_TAG_STRING); //code for string
        header1.push_back(0); // length of string
        header2 = "";
        header3.push_back(BOTTLE_TAG_LIST|BOTTLE_TAG_INT); // bottle code + specialisation with ints
        header3.push_back(0); // <- set the number of ints here (e.g. 2 * #v's)
        elementINTS = 0;
        elementBYTES = sizeof(int32_t) * elementINTS;
    }

    /// \brief set the type of event that this vBottleMimic will send
    void setHeader(std::string eventtype) {
        header1[3] = eventtype.size();  //set the string length
        header2 = eventtype;            //set the string itself

        elementINTS = packetSize(eventtype);
        elementBYTES = sizeof(int32_t) * elementINTS;

    }

    /// \brief for data already allocated in contiguous space. Just send this
    /// data on a port without memory reallocation.
    void setExternalData(const char * datablock, unsigned int datalength) {

        //no check for event-type safety is possible..

        header3[1] = elementINTS * (datalength / elementBYTES); //forced to be x8
        this->datablock = datablock;
        this->datalength = elementBYTES * header3[1] / elementINTS; //forced to be x8

    }

    /// \brief send an entire vQueue. The queue is encoded and allocated
    /// into a single contiguous memory space. Faster than a standard vBottle.
    void setInternalData(const vQueue &q) {

        if(header2 != q.front()->tag)
            setHeader(q.front()->tag);

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

    /// \brief send an entire vQueue. The queue is encoded and allocated
    /// into a single contiguous memory space. Faster than a standard vBottle.
    template <class T> void setInternalData(const std::deque<T> &q) {

        if(header2 != T::tag)
            setHeader(T::tag);

        header3[1] = elementINTS * q.size(); //number of ints

        if((int)internaldata.size() < header3[1]) //increase internal mem if needed
            internaldata.resize(header3[1]);

        unsigned int pos = 0;
        for(unsigned int i = 0; i < q.size(); i++)  //decode the data into
            q[i].encode(internaldata, pos);        //internal memeory

        if(pos != (unsigned int)header3[1])
            yError() << "vPortInterface: encoding incorrect";

        this->datablock = (const char *)internaldata.data();
        this->datalength = elementBYTES * q.size();
    }

    /// \brief write the data on the connection.
    bool write(yarp::os::ConnectionWriter& connection) const {

        connection.appendBlock((const char *)header1.data(),
                                       header1.size() * sizeof(int32_t));
        connection.appendBlock(header2.c_str(), header1[3]);
        connection.appendBlock((const char *)header3.data(),
                                       header3.size() * sizeof(int32_t));
        connection.appendBlock(datablock, datalength);

        return !connection.isError();
    }

    /// \brief does nothing as this is a write-only port.
    bool read(yarp::os::ConnectionReader& connection) {

        //META DATA OF BOTTLE
        if(connection.expectInt() != BOTTLE_TAG_LIST) //a list
            return false;
        if(connection.expectInt() != 2) //of two internal bottles
            return false;

        //DATA OF FIRST INTERNAL BOTTLE (type of event)
        if(connection.expectInt() != BOTTLE_TAG_STRING) // first of two
            return false;
        int str_len = connection.expectInt();
        event_type.resize(str_len);
        connection.expectBlock((char *)event_type.data(), str_len);

        //DATA OF SECOND INTERNAL BOTTLE (data of events)
        if(connection.expectInt() != (BOTTLE_TAG_LIST|BOTTLE_TAG_INT32))
            return false;
        ints_to_read = (unsigned int)connection.expectInt(); //in integers!!

        if(ints_to_read > internaldata.size())
            internaldata.resize(ints_to_read);
        if(!connection.expectBlock((char *)internaldata.data(),
                                   sizeof(std::int32_t) * ints_to_read)) {
            yError() << "Could not read datablock";
            return false;
        }

        return true;
    }



    bool decodePacket(vector<int32_t> &read_q)
    {
        read_q = internaldata;
        return true;
    }

    bool decodePacket(vQueue &read_q)
    {
        int event_size = packetSize(event_type);
        if(!event_size) {
            yError() << "Do not know event-type";
            return false;
        }

        event<> v = createEvent(event_type);
        if(v == nullptr) {
            yError() << "Do not know event-type";
            return false;
        }

        int *data = internaldata.data();
        for(unsigned int i = 0; i < ints_to_read / event_size; i++) {
            v->decode(data);
            read_q.push_back(v->clone());
        }
        return true;

    }

    template <class T> bool decodePacket(vector<T> &read_q)
    {

        if(event_type != T::tag) {
            yWarning() << "Incompatible event-type read";
            return false;
        }

        int *data = internaldata.data();
        read_q.resize(ints_to_read / packetSize(event_type));
        for(unsigned int i = 0; i < read_q.size(); i++) {
            read_q[i].decode(data);
        }
        return true;
    }


};

//this should open a yarp::os::Port
class vWritePort
{

protected:

    vPortableInterface internal_storage;
    Port port;

public:

    bool open(std::string name)
    {
        return port.open(name);
    }

    void close()
    {
        port.close();
    }

    void setWriteType(std::string tag)
    {
        internal_storage.setHeader(tag);
    }

    int getOutputCount() {
        return port.getOutputCount();
    }

    bool write(const vector<int32_t> &q, Stamp envelope)
    {
        internal_storage.setExternalData((const char *)q.data(),
                                         q.size() * sizeof(int32_t));

        if(!port.setEnvelope(envelope))
            return false;
        if(!port.write(internal_storage))
            return false;
        return true;


    }

    bool write(const vQueue &q, Stamp envelope)
    {
        internal_storage.setInternalData(q);
        if(!port.setEnvelope(envelope))
            return false;
        if(!port.write(internal_storage))
            return false;
        return true;

    }

    template <class T> bool write(const std::deque<T> &q, Stamp envelope)
    {
        internal_storage.setInternalData<T>(q);
        if(!port.setEnvelope(envelope))
            return false;
        if(!port.write(internal_storage))
            return false;
        return true;

    }

};

template <class T> class vReadPort : public Thread
{

protected:

    vPortableInterface internal_storage;
    Port port;

    deque< T* > qq;
    deque<Stamp> sq;
    T *working_queue;

    Mutex m;
    Mutex read_mutex;
    Semaphore dataavailable;

    unsigned int qlimit;
    unsigned int unprocdqs;
    unsigned int delay_nv;
    long unsigned int delay_t;
    double event_rate;


public:

    /// \brief constructor
    vReadPort()
    {
        qlimit = 0;
        delay_nv = 0;
        delay_t = 0;
        event_rate = 0;
        unprocdqs = 0;
        working_queue = nullptr;

        setPriority(99, SCHED_FIFO);

        dataavailable.wait(); //init counter to 0
    }

    /// \brief desctructor
    ~vReadPort()
    {
        m.lock();
        typename deque< T* >::iterator i;
        for(i = qq.begin(); i != qq.end(); i++)
            delete *i;
        qq.clear();
        m.unlock();
    }


    bool open(std::string name)
    {
        //port.setTimeout(1.0);
        if(!port.open(name)) {
            yError() << "Could not open vGenReadPort input port: " << name;
            return false;
        }
        start();
        return true;
    }

    void interrupt()
    {
        read_mutex.lock();
        port.interrupt();
    }

    void resume()
    {
        port.resume();
        read_mutex.unlock();
    }

    void close()
    {
        this->stop(); //make sure the isStopping() is true
        port.close(); //close the port connections
    }

    void onStop()
    {
        port.interrupt(); //port.read() will return false
        read_mutex.unlock(); //allow port.read() to be called
        dataavailable.post(); //all a this->read() to return
    }

    void run()
    {
        while(true) {

            //blocking read of data from the port
            read_mutex.lock();
            bool read_success = port.read(internal_storage);
            read_mutex.unlock();

            if(!read_success) {
                if(!isStopping())
                    yWarning() << "vGenReadPort read return false!";
                break;
            }

            if(qlimit && qq.size() >= qlimit)
                continue;

            yarp::os::Stamp yarp_stamp;
            port.getEnvelope(yarp_stamp);
            T *next_queue = new T;
            internal_storage.decodePacket(*next_queue);

            m.lock();

            qq.push_back(next_queue);
            sq.push_back(yarp_stamp);

            unprocdqs++;

            int q_events = countEvents<T>(*next_queue);
            int q_time = countTime<T>(*next_queue);

            delay_nv += q_events;
            delay_t += q_time;
            if(q_time)
                event_rate = q_events / (double)q_time;

            m.unlock();

            dataavailable.post();

        }

    }

    /// \brief ask for a pointer to the next vQueue. Blocks if no data is ready.
    const T* read(yarp::os::Stamp &yarpstamp)
    {
        if(working_queue) {
            m.lock();

            delay_nv -= countEvents<T>(*qq.front());
            delay_t -= countTime<T>(*qq.front());

            delete qq.front();
            qq.pop_front();
            sq.pop_front();
            m.unlock();
        }

        dataavailable.wait();

        if(qq.size()) {
            yarpstamp = sq.front();
            working_queue = qq.front();
            m.lock();
            unprocdqs--;
            m.unlock();
        }  else {
            working_queue =  0;
        }

        return working_queue;

    }

    /// \brief set the maximum number of qs that can be stored in the buffer.
    /// A value of 0 keeps all qs.
    void setQLimit(unsigned int number_of_qs)
    {
        qlimit = number_of_qs;
    }

    /// \brief unBlocks the blocking call in getNextQ. Useful to ensure a
    /// graceful shutdown. No guarantee the return of getNextQ will be valid.
    void releaseDataLock()
    {
        dataavailable.post();
    }

    /// \brief ask for the number of vQueues currently allocated.
    unsigned int queryunprocessed()
    {
        return unprocdqs;
    }

    /// \brief ask for the number of events in all vQueues.
    unsigned int queryDelayN()
    {
        return delay_nv;
    }

    /// \brief ask for the total time spanned by all vQueues.
    double queryDelayT()
    {
        return delay_t * vtsHelper::tsscaler;
    }

    /// \brief ask for the high precision event rate
    double queryRate()
    {
        return event_rate * vtsHelper::vtsscaler;
    }

    std::string delayStatString()
    {
        std::ostringstream oss;
        oss << "qs: " << queryunprocessed() << " events: " << queryDelayN() <<
               " time(s): " << queryDelayT() << " rate: " << queryRate();
        return oss.str();
    }

};

///// \brief an asynchronous reading port that accepts vBottles and decodes them
//class vGenReadPort : public yarp::os::Thread
//{
//protected:

//    vPortableInterface internal_storage;
//    Port port;

//    std::deque< vQueue* > qq;
//    std::deque<yarp::os::Stamp> sq;
//    vQueue *working_queue;

//    yarp::os::Mutex m;
//    yarp::os::Semaphore dataavailable;
//    Mutex read_mutex;

//    unsigned int qlimit;
//    unsigned int unprocdqs;
//    unsigned int delay_nv;
//    long unsigned int delay_t;
//    double event_rate;


//public:

//    /// \brief constructor
//    vGenReadPort()
//    {
//        qlimit = 0;
//        delay_nv = 0;
//        delay_t = 0;
//        event_rate = 0;
//        unprocdqs = 0;
//        working_queue = nullptr;

//        setPriority(99, SCHED_FIFO);

//        dataavailable.wait(); //init counter to 0
//    }

//    /// \brief desctructor
//    ~vGenReadPort()
//    {
//        m.lock();
//        std::deque< vQueue* >::iterator i;
//        for(i = qq.begin(); i != qq.end(); i++)
//            delete *i;
//        qq.clear();
//        m.unlock();
//    }

//    bool open(std::string name)
//    {
//        //port.setTimeout(1.0);
//        if(!port.open(name)) {
//            yError() << "Could not open vGenReadPort input port: " << name;
//            return false;
//        }
//        start();
//        return true;
//    }

//    void interrupt()
//    {
//        read_mutex.lock();
//        port.interrupt();
//    }

//    void resume()
//    {
//        port.resume();
//        read_mutex.unlock();
//    }

//    void close()
//    {
//        this->stop(); //make sure the isStopping() is true
//        port.close(); //close the port connections
//    }

//    void onStop()
//    {
//        port.interrupt(); //port.read() will return false
//        read_mutex.unlock(); //allow port.read() to be called
//        dataavailable.post(); //all a this->read() to return
//    }

//    void run()
//    {
//        while(true) {

//            vQueue *next_queue = new vQueue;
//            internal_storage.setReadContainer(*next_queue);

//            read_mutex.lock();
//            if(!port.read(internal_storage)) {
//                read_mutex.unlock();
//                if(!isStopping())
//                    yWarning() << "vGenReadPort read return false!";
//                delete next_queue;
//                break;

//            }
//            read_mutex.unlock();

//            yarp::os::Stamp yarp_stamp;
//            port.getEnvelope(yarp_stamp);

//            if(qlimit && qq.size() >= qlimit) {
//                delete next_queue;
//                continue;
//            }

//            m.lock();

//            qq.push_back(next_queue);
//            sq.push_back(yarp_stamp);

//            unprocdqs++;

//            delay_nv += qq.back()->size();
//            int dt = qq.back()->back()->stamp - qq.back()->front()->stamp;
//            if(dt < 0) dt += vtsHelper::max_stamp;
//            delay_t += dt;
//            if(dt)
//                event_rate = qq.back()->size() / (double)dt;

//            m.unlock();
//            dataavailable.post();

//        }

//    }

//    /// \brief ask for a pointer to the next vQueue. Blocks if no data is ready.
//    const vQueue* read(yarp::os::Stamp &yarpstamp)
//    {
//        if(working_queue) {
//            m.lock();

//            delay_nv -= qq.front()->size();
//            int dt = qq.front()->back()->stamp - qq.front()->front()->stamp;
//            if(dt < 0) dt += vtsHelper::max_stamp;
//            delay_t -= dt;

//            delete qq.front();
//            qq.pop_front();
//            sq.pop_front();
//            m.unlock();
//        }

//        dataavailable.wait();

//        if(qq.size()) {
//            yarpstamp = sq.front();
//            working_queue = qq.front();
//            m.lock();
//            unprocdqs--;
//            m.unlock();
//        }  else {
//            working_queue =  0;
//        }

//        return working_queue;

//    }

//    /// \brief set the maximum number of qs that can be stored in the buffer.
//    /// A value of 0 keeps all qs.
//    void setQLimit(unsigned int number_of_qs)
//    {
//        qlimit = number_of_qs;
//    }

//    /// \brief unBlocks the blocking call in getNextQ. Useful to ensure a
//    /// graceful shutdown. No guarantee the return of getNextQ will be valid.
//    void releaseDataLock()
//    {
//        dataavailable.post();
//    }

//    /// \brief ask for the number of vQueues currently allocated.
//    unsigned int queryunprocessed()
//    {
//        return unprocdqs;
//    }

//    /// \brief ask for the number of events in all vQueues.
//    unsigned int queryDelayN()
//    {
//        return delay_nv;
//    }

//    /// \brief ask for the total time spanned by all vQueues.
//    double queryDelayT()
//    {
//        return delay_t * vtsHelper::tsscaler;
//    }

//    /// \brief ask for the high precision event rate
//    double queryRate()
//    {
//        return event_rate * vtsHelper::vtsscaler;
//    }

//    std::string delayStatString()
//    {
//        std::ostringstream oss;
//        oss << "qs: " << queryunprocessed() << " events: " << queryDelayN() <<
//               " time(s): " << queryDelayT() << " rate: " << queryRate();
//        return oss.str();
//    }

//};


//template <class T> class vReadPort : private vGenReadPort
//{
//protected:

//    vPortInterface<T> internal_storage;
//    std::deque< std::vector<T>* > qq;
//    std::vector<T> *working_queue;

//public:

//    /// \brief constructor
//    vReadPort() : vGenReadPort()
//    {
//        working_queue = nullptr;
//    }

//    /// \brief desctructor
//    ~vReadPort()
//    {

//        m.lock();
//        typename std::deque< std::vector<T>* >::iterator i;
//        for(i = qq.begin(); i != qq.end(); i++)
//            delete *i;
//        qq.clear();
//        m.unlock();
//    }

//    void run()
//    {
//        while(!isStopping()) {

//            std::vector<T> *next_queue = new std::vector<T>;
//            internal_storage.setReadContainer(*next_queue);
//            //internal_storage.setReadQueue(*next_queue);
//            read_mutex.lock();
//            if(!port.read(internal_storage)) {
//                read_mutex.unlock();
//                if(!isStopping())
//                    yWarning() << "vReadPort<> read return false!";
//                delete next_queue;
//                break;
//            }
//            read_mutex.unlock();

//            yarp::os::Stamp yarp_stamp;
//            port.getEnvelope(yarp_stamp);

//            if(qlimit && qq.size() >= qlimit) {
//                delete next_queue;
//                continue;
//            }

//            m.lock();

//            qq.push_back(next_queue);
//            sq.push_back(yarp_stamp);

//            unprocdqs++;

//            delay_nv += qq.back()->size();
//            int dt = qq.back()->back().stamp - qq.back()->front().stamp;
//            if(dt < 0) dt += vtsHelper::max_stamp;
//            delay_t += dt;
//            if(dt)
//                event_rate = qq.back()->size() / (double)dt;
//            m.unlock();

//            //if getNextQ is blocking - let it get the new data
//            dataavailable.post();

//        }



//    }

//    /// \brief ask for a pointer to the next vQueue. Blocks if no data is ready.
//    const std::vector<T>* read(yarp::os::Stamp &yarpstamp)
//    {

//        if(working_queue) {
//            m.lock();

//            delay_nv -= qq.front()->size();
//            int dt = qq.front()->back().stamp - qq.front()->front().stamp;
//            if(dt < 0) dt += vtsHelper::max_stamp;
//            delay_t -= dt;

//            delete qq.front();
//            qq.pop_front();
//            sq.pop_front();
//            m.unlock();
//        }

//        dataavailable.wait();

//        if(qq.size()) {
//            yarpstamp = sq.front();
//            working_queue = qq.front();
//            m.lock();
//            unprocdqs--;
//            m.unlock();
//        }  else {
//            working_queue =  0;
//        }
//        return working_queue;

//    }

//    using vGenReadPort::open;
//    using vGenReadPort::close;
//    using vGenReadPort::setQLimit;
//    using vGenReadPort::queryunprocessed;
//    using vGenReadPort::releaseDataLock;
//    using vGenReadPort::queryDelayN;
//    using vGenReadPort::queryDelayT;
//    using vGenReadPort::queryRate;
//    using vGenReadPort::delayStatString;
//    using vGenReadPort::interrupt;
//    using vGenReadPort::resume;

//};

} //end namespace ev

#endif
