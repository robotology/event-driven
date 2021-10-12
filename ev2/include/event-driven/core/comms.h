/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
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

#ifndef __VCODEC__
#define __VCODEC__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Portable.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <vector>
#include <deque>
#include <list>
#include <cstring>
#include <unistd.h>
#include <mutex>
//#include <type_traits>

namespace ev {

typedef struct
{
    unsigned int count;
    double duration;
} info;

typedef struct timeStamp {
    static const std::string tag;
    unsigned int ts:31;
    unsigned int _fill:1;
} timeStamp;
using TS = timeStamp;

typedef struct addressEvent : public timeStamp {
    static const std::string tag;
    unsigned int p:1;
    unsigned int x:10;
    unsigned int _xfill:1;
    unsigned int y:9;
    unsigned int corner:1;
    unsigned int channel:1;
    unsigned int type:1;
    unsigned int skin:1;
    unsigned int _fill:7;
} addressEvent;
using AE = addressEvent;

typedef struct skinEvent {
    static const std::string tag;
    unsigned int polarity:1;
    unsigned int taxel:10;
    unsigned int _reserved1:2;
    unsigned int cross_base:1;
    unsigned int _sample:1;
    unsigned int _error:1;
    unsigned int body_part:3;
    unsigned int _reserved2:3;
    unsigned int side:1;
    unsigned int type:1;
    unsigned int skin:1;
    unsigned int _fill:7;
} skinEvent;

typedef struct skinSample {
    static const std::string tag;
    unsigned int _ts:31;
    unsigned int _fill1:1;
    unsigned int value:16;
    unsigned int _fill2:16;
} skinSample;

/// \brief an AddressEvent with a velocity in visual space
typedef struct flowEvent {
    static const std::string tag;
    float vx;
    float vy;
} flowEvent;

/// \brief a LabelledAE with parameters that define a 2D gaussian
typedef struct gaussianEvent {
    static const std::string tag;
    float sigx;
    float sigy;
    float sigxy;
} gaussianEvent;

/// \brief an event with a pixel location, camera number and polarity
typedef struct imuSample {
    static const std::string tag;
    int value:16;
    unsigned int sensor:4;
    unsigned int _r1:2;
    unsigned int channel:1;
    unsigned int type:1;
    unsigned int _r2:8;
} imuSample;

typedef struct neuronEvent : public timeStamp {
    static const std::string tag;
    int id;
} neuronEvent;

template <typename T> class packet : public yarp::os::Portable {

private:
    //TODO: this buffer size should be set by a compile time variable (cmake)
    static const unsigned int initial_buffer_size{1048576};
    unsigned int n_elements{0};
    std::vector<T> buffer;
    double _duration{0.0};

    bool invalidPacket(const std::string &msg) const
    {
        yError() << "Invalid Packet:" << msg;
        return false;
    }

public:

    packet(void)
    {
        buffer.resize(initial_buffer_size);
    }

    // template<class Q = T>
    // typename std::enable_if<std::is_same<Q, int32_t>::value, bool>::type someFunc(std::string s)
    // {
    //     return true;
    // }

    // template<class Q = T>
    // typename std::enable_if<!std::is_same<Q, int32_t>::value, bool>::type someFunc(std::string s)
    // {
    //     return Q::tag == s;
    // }

    bool read(yarp::os::ConnectionReader &reader) override
    {
        int32_t r = reader.expectInt32();
        if(r == 0) return false;
        else if(r != BOTTLE_TAG_LIST) return invalidPacket("not a list");
        if(reader.expectInt32() != 3) return invalidPacket("not 3 elements");
        if(reader.expectInt32() != BOTTLE_TAG_STRING) return invalidPacket("no tag");
        if(reader.expectString() != T::tag) return invalidPacket("incorrect tag");
        if(reader.expectInt32() != BOTTLE_TAG_INT32) return invalidPacket("no duration");
        _duration = reader.expectInt32() * 0.000001;
        if(reader.expectInt32() != BOTTLE_TAG_STRING) return invalidPacket("no data");
        int n = reader.expectInt32(); // STRING_LENGTH
        if(n % sizeof(T)) return invalidPacket("data invalid length");
        n_elements = n / sizeof(T);
        if(buffer.size() < n_elements) buffer.resize(n_elements);
        return reader.expectBlock((char *)buffer.data(), n);
    }

    bool write(yarp::os::ConnectionWriter &writer) const override
    {
        if(!(_duration > 0.0)) {
            yError() << "ev::packet::write() : no duration";
            return true;
        }

        writer.appendInt32(BOTTLE_TAG_LIST);
        writer.appendInt32(3);
        writer.appendInt32(BOTTLE_TAG_STRING);
        writer.appendInt32(T::tag.length());
        writer.appendExternalBlock(T::tag.c_str(), T::tag.length());
        writer.appendInt32(BOTTLE_TAG_INT32);
        writer.appendInt32((int)(_duration * 1000000 + 0.5));
        writer.appendInt32(BOTTLE_TAG_STRING);
        writer.appendInt32(n_elements * sizeof(T));
        writer.appendExternalBlock((char *)buffer.data(), n_elements * sizeof(T));
        return !writer.isError();
    }

    void clear(void)
    {
        n_elements = 0;
        _duration = 0.0;
    }

    void push_back(const T &element)
    {
        if(buffer.size() <= n_elements)
            buffer.resize(buffer.size() * 2);
        buffer[n_elements++] = element;
    }

    using iterator = typename std::vector<T>::iterator;

    typename std::vector<T>::iterator begin()
    {
        return buffer.begin();
    }

    typename std::vector<T>::iterator end()
    {
        return buffer.begin() + n_elements; //is this dynamic?
    }

    T& operator[](std::size_t index)
    {
        return buffer[index];
    }

    size_t size(void) const
    {
        return n_elements;
    }

    void size(int n)
    {
        buffer.resize(n);
    }

    void duration(const double &seconds)
    {
        _duration = seconds;
    }

    double duration(void) const
    {
        return _duration;
    }

    int fillFromDevice(const int fd, const int min_packet_size, const int max_packet_size)
    {
        if(buffer.size() * sizeof(T) < max_packet_size)
            buffer.resize(max_packet_size / sizeof(T) + (max_packet_size % sizeof(T) ? 1 : 0));

        int r = min_packet_size;
        unsigned int n_bytes_read = 0;
        while(r >= min_packet_size && n_bytes_read < max_packet_size) {
            r = ::read(fd, (char *)buffer.data() + n_bytes_read, max_packet_size - n_bytes_read);
            if(r < 0)
                yInfo() << "[READ ]" << std::strerror(errno);
            else
                n_bytes_read += r;
        }

        if(n_bytes_read % sizeof(T))
            yError() << "[READ ] partial read. bad fault. get help.";

        n_elements = n_bytes_read / sizeof(T);

        return n_bytes_read;
    }

    int singleDeviceRead(const int fd) 
    {
        int max_packet_size = buffer.size() * sizeof(T);
        int n_bytes_read = this->size() * sizeof(T);
        
        int r = -1;
        while(r < 0) 
        { 
            r = ::read(fd, (char *)buffer.data() + n_bytes_read, max_packet_size - n_bytes_read);
            if(r < 0)
                yInfo() << "[READ ]" << std::strerror(errno);
        }
        if (r % sizeof(T))
            yError() << "[READ ] partial read. bad fault. get help.";
        else
            n_elements += r / sizeof(T);

        return r;
    }

    int pushToDevice(int fd) const
    {
        
        //move to bytes space
        size_t bytes_to_write = n_elements * sizeof(T);
        size_t written = 0;
        while(written < bytes_to_write) {

            int r = ::write(fd, (char *)buffer.data() + written, bytes_to_write - written);

            if(r > 0) { //success!
                written += r;
            } else if(r < 0 && errno != EAGAIN) { //error!
                yError() << "[WRITE]" << std::strerror(errno);
                break;
            }
        }
        return written;
    }

};

/// vPortWrapper forces uses in a way to avoid
template <typename T> class BufferedPort : protected yarp::os::BufferedPort< ev::packet<T> >
{
public:

    BufferedPort()
    {
        yarp::os::BufferedPort< ev::packet<T> >::setStrict();
    }

    void write()
    {
        //we don't really want packets to "build up" in the outgoing thread.
        //if that happens we want to address the problem elsewhere. but we do
        //want to use a secondary thread to send the data (and limit buffer
        //swapping to 2). This code should do that.
        yarp::os::BufferedPort< ev::packet<T> >::waitForWrite(); 
        yarp::os::BufferedPort< ev::packet<T> >::writeStrict();
    }

    ev::packet<T>& prepare() 
    {
        auto &p = yarp::os::BufferedPort< ev::packet<T> >::prepare();
        p.clear();
        return p;
    }

    using yarp::os::BufferedPort< ev::packet<T> >::open;
    using yarp::os::BufferedPort< ev::packet<T> >::getPendingReads;
    using yarp::os::BufferedPort< ev::packet<T> >::read;
    using yarp::os::BufferedPort< ev::packet<T> >::unprepare;
    using yarp::os::BufferedPort< ev::packet<T> >::setEnvelope;
    using yarp::os::BufferedPort< ev::packet<T> >::getEnvelope;
    using yarp::os::BufferedPort< ev::packet<T> >::close;
    using yarp::os::BufferedPort< ev::packet<T> >::interrupt;
    using yarp::os::BufferedPort< ev::packet<T> >::isWriting;
};

template <typename T> class window : public yarp::os::Thread
{
public:

    struct iterator
    {
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = std::ptrdiff_t;
        using value_type        = T;
        using pointer           = T*;
        using reference         = T&;

        iterator() : m_ptr(nullptr) {}
        void setAsEnd(typename std::list< packet<T>* >::iterator last)
        {
            m_ptr = (**last).end();
        }

        void setAsStart(typename std::list< packet<T>* >::iterator first, typename std::list< packet<T>* >::iterator last)
        {
            m_ptr = (**first).begin();
            packet_it = first;
            final = last;
        }

        T& operator*() const { return *m_ptr; }
        T* operator->() { return m_ptr; }
        iterator& operator++()
        {

            m_ptr++;
            if(m_ptr == (*packet_it)->end() && packet_it != final) {
                m_ptr = (*(++packet_it))->begin();
            }

            return *this;
        }
        //Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }
        friend bool operator== (const iterator& a, const iterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const iterator& a, const iterator& b) { return a.m_ptr != b.m_ptr; };

    private:
        typename packet<T>::iterator m_ptr;
        typename std::list< packet<T>* >::iterator packet_it;
        typename std::list< packet<T>* >::iterator final;
    };

    iterator begin() { return _begin; }
    iterator end()   { return _end; }

    //different methods to access the data
    //all data and delete it once read. set the begin() and end() iterators
    //set the duration and number of events
    //when this is called delete all data before begin().
    info readAll(bool blocking = true)
    {
        if(blocking) data_available.wait();
        m.lock();

        //first of all remove all the old stuff
        _removeAlreadyRead();

        //set the new window for all data
        _setAllIterators(active.begin(), active.end());

        info ret{_count, _duration};
        m.unlock();

        return ret;

    }

    info readSlidingWinT(double seconds, bool blocking = true)
    {
        if(blocking) data_available.wait();
        m.lock();

        //pop packets until we find the desired temporal window
        while(_duration > seconds) {
            auto packet_duration = (**active.begin()).duration();
            if(_duration - packet_duration < seconds)
                break;
            _duration -= packet_duration;
            _count -= (**active.begin()).size();
            inactive.push_back(*active.begin());
            active.pop_front();
        }


        //set the correct iterators
        _setAllIterators(active.begin(), active.end());

        info ret{_count, _duration};
        m.unlock();

        return ret;
    }

    info readSlidingWinN(unsigned int count, bool blocking = true)
    {
        if(blocking) data_available.wait();
        m.lock();

        //pop packets until we find the desired temporal window
        while(_count > count) {
            auto packet_count = (**active.begin()).size();
            if(_count - packet_count < count)
                break;
            _count -= packet_count;
            _duration -= (**active.begin()).duration();
            inactive.push_back(*active.begin());
            active.pop_front();
        }

        //set the correct iterators
        _setAllIterators(active.begin(), active.end());

        info ret{_count, _duration};
        m.unlock();

        return ret;
    }

    info readChunkN(unsigned int count, bool blocking = true)
    {
        m.lock();

        //first of all remove all the old stuff
        _removeAlreadyRead();

        //if we are blocking on a condition then wait till we have enough data
        if(blocking) {
            while(_count < count) {
                m.unlock();
                data_available.wait();
                m.lock();
                if(isStopping())
                    break;
            }
        }

        //move the iterator until we find our condition, or no more data
        info ret{0, 0};
        last_packet = active.begin();
        while(last_packet != active.end()) {
            ret.duration += (**last_packet).duration();
            ret.count += (**last_packet).size();
            last_packet++;

            if(ret.count >= count)
                break;
        }
        if(last_packet == active.end())
            data_available.check(); //by chance all data gone -> lock()

        //set the new window for all data
        _setAllIterators(active.begin(), last_packet);

        m.unlock();

        return ret;
    }

    info readChunkT(float seconds, bool blocking = true)
    {
        m.lock();

        //first of all remove all the old stuff
        _removeAlreadyRead();

        //if we are blocking on a condition then wait till we have enough data
        if(blocking) {
            while(_duration < seconds) {
                m.unlock();
                data_available.wait();
                m.lock();
                if(isStopping())
                    break;
            }
        }

        //move the iterator until we find our condition, or no more data
        info ret{0, 0};
        last_packet = active.begin();
        while(last_packet != active.end()) {
            ret.duration += (**last_packet).duration();
            ret.count += (**last_packet).size();
            last_packet++;
            if(ret.duration >= seconds)
                break;
        }
        if(last_packet == active.end())
            data_available.check(); //by chance all data gone -> lock()

        //set the new window for all data
        _setAllIterators(active.begin(), last_packet);

        m.unlock();

        return ret;
    }

    window()
    {
        first_packet = active.begin();
        last_packet = active.begin();
        data_available = yarp::os::Semaphore(0);
    }

    ~window()
    {
        stop();
        port.close();
        for(auto i = active.begin(); i != active.end(); i++)
            delete *i;
        for(auto i = inactive.begin(); i != inactive.end(); i++)
            delete *i;
    }

    double duration(void) const
    {
        return _duration;
    }

    unsigned int count(void) const
    {
        return _count;
    }

    bool open(const std::string name)
    {
        if(!port.open(name)) {
            yError() << "Could not open port: " << name;
            return false;
        }
        return this->start();
    }

    void interrupt()
    {
        port.interrupt();
    }

    void resume()
    {
        port.resume();
    }

    void onStop()
    {
        port.close();
        data_available.post();
        m.unlock();
    }

    void run()
    {
        while(!isStopping()) {

            //blocking read of data from the port
            packet<T>* current_packet = nullptr;
            if(inactive.empty()) {
                current_packet = new packet<T>;
            } else {
                current_packet = inactive.back();
                m.lock();
                inactive.pop_back();
                m.unlock();
            }


            bool read_success = port.read(*current_packet);

            if(!read_success && !isStopping()) {
                yWarning() << "port read failure!";
                break;
            }

            m.lock();
            active.push_back(current_packet);
            _duration += current_packet->duration();
            _count += current_packet->size();
            m.unlock();
            data_available.check(); //we would prefer to use a binary_semaphore here
            data_available.post();
        }

    }



private:

    void _removeAlreadyRead(void)
    {
        if(last_packet != active.end()) ++last_packet;
        while(first_packet != last_packet) {
            _duration -= (**first_packet).duration();
            _count -= (**first_packet).size();
            inactive.push_back(*first_packet);
            first_packet = active.erase(first_packet);
        }
    }

    void _setAllIterators(typename std::list< packet<T>* >::iterator start, typename  std::list< packet<T>* >::iterator end)
    {
        if(active.empty()) {
            first_packet = active.begin();
            last_packet = active.begin();
            _begin = iterator();
            _end = iterator();
        } else {
            first_packet = start;
            last_packet = std::prev(end); //we need to drop it back one so it is inclusive in the data
            _begin.setAsStart(first_packet, last_packet);
            _end.setAsEnd(last_packet);
        }
    }

    yarp::os::Port port;

    std::list< packet<T>* > active;
    std::list< packet<T>* > inactive;
    typename std::list< packet<T>* >::iterator last_packet;
    typename std::list< packet<T>* >::iterator first_packet;
    iterator _begin;
    iterator _end;
    double _duration{0.0f};
    unsigned int _count{0};
    std::mutex m;
    yarp::os::Semaphore data_available;


};



}

#endif


