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

#pragma once

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Portable.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <vector>
#include <deque>
#include <list>
#include <cstring>
#include <unistd.h>
#include <mutex>
#include <iomanip>
#include <condition_variable>
#include <fstream>

namespace ev {

typedef struct
{
    unsigned int count;
    double duration;
    double timestamp;
} info;


template <typename T> class packet : public yarp::os::Portable {

private:
    unsigned int n_elements{0};
    std::vector<T> buffer;
    double _duration{0.0};
    yarp::os::Stamp e;

    bool invalidPacket(const std::string &msg) const
    {
        yError() << "Invalid Packet:" << msg;
        return false;
    }

public:

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
            buffer.resize(buffer.size() + 16384);
        buffer[n_elements++] = element;
    }

    using iterator = typename std::vector<T>::iterator;

    typename std::vector<T>::iterator begin()
    {
        return buffer.begin();
    }

    typename std::vector<T>::iterator end()
    {
        return buffer.begin() + n_elements;
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

    inline const double timestamp() const
    {
        return e.getTime();
    }

    inline const int id() const
    {
        return e.getCount();
    }

    inline yarp::os::Stamp& envelope()
    {
        return e;
    }

    int fillFromMemory(char *s, int bytes)
    {
        n_elements = bytes / sizeof(T);
        buffer.resize(n_elements);
        memcpy((char *)buffer.data(), s, n_elements * sizeof(T));
        return n_elements * sizeof(T);
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
private:
    ev::packet<T> *prepared = nullptr;
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
        if(!prepared) 
        {
            yError() << "internal prepare state mismatch in ev::BufferedPort. "
                        "Nothing written";
            return;
        }
        yarp::os::BufferedPort< ev::packet<T> >::setEnvelope(prepared->envelope());
        yarp::os::BufferedPort< ev::packet<T> >::waitForWrite(); 
        yarp::os::BufferedPort< ev::packet<T> >::writeStrict();
        prepared = nullptr;
    }

    ev::packet<T>& prepare() 
    {
        auto &p = yarp::os::BufferedPort< ev::packet<T> >::prepare();
        p.clear();
        prepared = &p;
        return p;
    }

    bool unprepare()
    {
        prepared = nullptr;
        return yarp::os::BufferedPort< ev::packet<T> >::unprepare();
    }

    ev::packet<T>* read(bool shouldWait = true) 
    {
        ev::packet<T>* result = yarp::os::BufferedPort<ev::packet<T> >::read(shouldWait);
        if(result) yarp::os::BufferedPort< ev::packet<T> >::getEnvelope(result->envelope());
        return result;
    }

    using yarp::os::BufferedPort< ev::packet<T> >::open;
    using yarp::os::BufferedPort< ev::packet<T> >::getPendingReads;
    using yarp::os::BufferedPort< ev::packet<T> >::close;
    using yarp::os::BufferedPort< ev::packet<T> >::interrupt;
    using yarp::os::BufferedPort< ev::packet<T> >::resume;
    using yarp::os::BufferedPort< ev::packet<T> >::isWriting;
    using yarp::os::BufferedPort< ev::packet<T> >::isClosed;
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
            _timestamp = (*last)->timestamp();
            _id = (*last)->id();
        }

        void setAsStart(typename std::list< packet<T>* >::iterator first, typename std::list< packet<T>* >::iterator last)
        {
            m_ptr = (**first).begin();
            packet_it = first;
            final = last;
            _timestamp = (*first)->timestamp();
            _id = (*first)->id();
        }

        //here we always return the "packet timestamp" if the user wants to use
        //the individual event timestamp, leave it to them to access it from the
        //pointer.
        double timestamp() 
        {
            return _timestamp;
        }

        double packetID()
        {
            return _id;
        }

        T& operator*() const { return *m_ptr; }
        T* operator->() { return &(*m_ptr); }
        iterator& operator++()
        {

            m_ptr++;
            if(m_ptr == (*packet_it)->end() && packet_it != final) {
                m_ptr = (*(++packet_it))->begin();
                _timestamp = (*packet_it)->timestamp();
                _id = (*packet_it)->id();
            }

            return *this;
        }

        iterator& operator++(int k)
        {
            //TODO - check this is a good method (this += k?)
            //for (auto i = 0; i < k; i++) {
                m_ptr++;
                if (m_ptr == (*packet_it)->end() && packet_it != final) {
                    m_ptr = (*(++packet_it))->begin();
                    _timestamp = (*packet_it)->timestamp();
                    _id = (*packet_it)->id();
                }
            //}

            return *this;
        }

        //Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }
        friend bool operator== (const iterator& a, const iterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const iterator& a, const iterator& b) { return a.m_ptr != b.m_ptr; };

        private:
        int _id{0};
        double _timestamp{0.0};
        typename packet<T>::iterator m_ptr;
        typename std::list< packet<T>* >::iterator packet_it;
        typename std::list< packet<T>* >::iterator final;
    };

    iterator begin() { return _begin; }
    iterator end()   { return _end; }

    //methods to access the data
    //read packet is unique as it returns the packet while the other methods
    //set the iterators that allow iteration through the data agnostic to the 
    //number of packets in the window
    packet<T>* readPacket(bool blocking = true)
    {
        std::unique_lock<std::mutex> lk(m);
        _removeAlreadyRead();

        if(blocking)
            signal.wait(lk, [this]{return in_port.count > 0;});

        if(active.empty()) 
        {
            _resetIterators(active.begin(), active.begin());
            in_window = {0, 0, 0};
            return nullptr;
        } 
        else
        {
            _resetIterators(active.begin(), std::next(active.begin()));
            in_window = {(unsigned int)(**active.begin()).size(), 
                        (**active.begin()).duration(), 
                        (**active.begin()).timestamp()};
            return *active.begin();
        }
    }

    info readAll(bool blocking = true)
    {
        //enter critical section
        std::unique_lock<std::mutex> lk(m);

        //remove all the old data
        _removeAlreadyRead();

        //if blocking wait for some data
        if(blocking)
            signal.wait(lk, [this]{return in_port.count > 0 || isStopping();});

        //set the new window for all data
        _resetIterators(active.begin(), active.end());
        in_window = in_port;

        return in_window;

    }

    info readSlidingWinT(double seconds, bool blocking = true)
    {
        std::unique_lock<std::mutex> lk(m);

        if(blocking) 
            signal.wait(lk, [this]{return in_port.count > in_window.count || isStopping();});
        
         //pop packets until we find the desired temporal window
        while(!active.empty())
        {
            //get the first packet in the active queue stats
            auto packet_duration = (**active.begin()).duration();
            auto packet_count =  (**active.begin()).size();

            //if we can remove the packet and stay in the correct time do it
            if(in_port.duration - packet_duration < seconds)
                break;
            in_port.duration -= packet_duration;
            in_port.count -= packet_count;
            inactive.push_back(*active.begin());
            active.pop_front();
        }

        //set the correct iterators
        _resetIterators(active.begin(), active.end());
        in_window = in_port;

        return in_window;
    }

    info readSlidingWinT(double seconds, double exact_time)
    {
        //ensure that time has passed in this port
        std::unique_lock<std::mutex> lk(m);
        signal.wait(lk, [this, &exact_time]{return (in_port.count && in_port.timestamp >= exact_time) || isStopping();});

         //pop packets until we find the desired temporal window less than the exact time
        while(!active.empty())
        {
            //get the first packet in the active queue stats
            auto packet_duration = (**active.begin()).duration();
            auto packet_count =  (**active.begin()).size();
            auto packet_timestamp = (**active.begin()).timestamp();

            //if we can remove the packet and stay in the correct time do it
            if(packet_timestamp + seconds >= exact_time)
                break;
            in_port.duration -= packet_duration;
            in_port.count -= packet_count;
            inactive.push_back(*active.begin());
            active.pop_front();
        }

        in_window = {0, 0.0, 0.0};
        auto i = active.begin();
        do {
            in_window.duration += (**i).duration();
            in_window.count +=  (**i).size();
            in_window.timestamp = (**i).timestamp();
            i++;
        } while(i != active.end() && (**i).timestamp() < exact_time);

        //set the correct iterators
        _resetIterators(active.begin(), i);

        return in_window;
    }

    info readSlidingWinN(unsigned int count, bool blocking = true)
    {
        std::unique_lock<std::mutex> lk(m);
        if(blocking) 
            signal.wait(lk, [this]{return  in_port.count > in_window.count || isStopping();});

         //pop packets until we find the desired fixed-count window
        while(!active.empty())
        {
            //get the first packet in the active queue stats
            auto packet_duration = (**active.begin()).duration();
            auto packet_count =  (**active.begin()).size();

            //if we can remove the packet and stay in the correct time do it
            if(in_port.count - packet_count < count)
                break;
            in_port.duration -= packet_duration;
            in_port.count -= packet_count;
            inactive.push_back(*active.begin());
            active.pop_front();
        }

        //set the correct iterators
        _resetIterators(active.begin(), active.end());
        in_window = in_port;

        return in_window;
    }

    info readChunkN(unsigned int count, bool blocking = true)
    {
        //first of all remove all the old stuff
        std::unique_lock<std::mutex> lk(m);
        _removeAlreadyRead();

        //if we are blocking on a condition then wait till we have enough data
        if(blocking) {
            while(in_port.count < count) {
                signal.wait(lk);
                if(isStopping()) return {0, 0, 0};
            }
        }

        //move the iterator until we find our condition, or no more data
        in_window = {0, 0, 0};
        last_packet = active.begin();
        while(last_packet != active.end()) {
            in_window.duration += (**last_packet).duration();
            in_window.count += (**last_packet).size();
            in_window.timestamp = (**last_packet).timestamp();
            last_packet++;

            if(in_window.count >= count)
                break;
        }

        //set the new window for all data
        _resetIterators(active.begin(), last_packet);

        return in_window;
    }

    info readChunkT(float seconds, bool blocking = true)
    {
        //first of all remove all the old stuff
        std::unique_lock<std::mutex> lk(m);
        _removeAlreadyRead();

        //if we are blocking on a condition then wait till we have enough data
        if(blocking) {
            while(in_port.duration < seconds) {
                signal.wait(lk);
                if(isStopping()) return {0, 0, 0};
            }
        }

        //move the iterator until we find our condition, or no more data
        in_window = {0, 0, 0};
        last_packet = active.begin();
        while(last_packet != active.end()) {
            in_window.duration += (**last_packet).duration();
            in_window.count += (**last_packet).size();
            in_window.timestamp = (**last_packet).timestamp();
            last_packet++;

            if(in_window.duration >= seconds)
                break;
        }

        //set the new window for all data
        _resetIterators(active.begin(), last_packet);

        return in_window;
    }

    window()
    {
        first_packet = active.begin();
        last_packet = active.begin();
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

    info stats_current(void) const
    {
        return in_window;
    }

    info stats_unprocessed(void) const
    {
        return {in_port.count - in_window.count,
                in_port.duration - in_window.duration,
                in_port.timestamp};
    }

    info stats_all(void) const
    {
        return in_port;
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
        //m.unlock();
    }

    void run()
    {
        while(true) {

            //find the best memory location to use or create more
            bool reused_inactive = false;
            packet<T>* current_packet = nullptr;
            if(inactive.empty()) {
                current_packet = new packet<T>;
            } else {    
                current_packet = inactive.front();
                reused_inactive = true;
            }

            //blocking read from the port
            bool read_success = port.read(*current_packet);

            //and handle return without data
            if(isStopping()) {
                break;
            }
            else if(!read_success) {
                yWarning() << "port read failure!";
                break;
            }

            port.getEnvelope(current_packet->envelope());

            //shared section - updated active, inactive, in_port info
            {
                std::lock_guard<std::mutex> lock(m); //releases leaving { }
                if (reused_inactive) inactive.pop_front();
                active.push_back(current_packet);
                in_port.duration += current_packet->duration();
                in_port.count += current_packet->size();
                in_port.timestamp = current_packet->timestamp();
            }
            signal.notify_one();
        }
        signal.notify_one();

    }

    std::string getName()
    {
        return port.getName();
    }

    int getInputCount()
    {
        return port.getInputCount();
    }


private:

    void _removeAlreadyRead(void)
    {
        if(last_packet != active.end()) ++last_packet;
        while(first_packet != last_packet) {
            in_port.duration -= (**first_packet).duration();
            in_port.count -= (**first_packet).size();
            inactive.push_back(*first_packet);
            first_packet = active.erase(first_packet);
        }
    }

    void _resetIterators(typename std::list< packet<T>* >::iterator start, typename  std::list< packet<T>* >::iterator end)
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

    //input port
    yarp::os::Port port;

    //data storage
    std::list< packet<T>* > active;
    std::list< packet<T>* > inactive;

    //in_port is all data that has been read
    info in_port{0};
    //in_window is data that is actively asked to be interated through
    info in_window{0};

    //packet iterators point to packets be "in_window"
    typename std::list< packet<T>* >::iterator last_packet;
    typename std::list< packet<T>* >::iterator first_packet;
    //iterators point to individual events "in_window"
    iterator _begin;
    iterator _end;

    //thread synchronisation
    std::mutex m;
    std::condition_variable signal;

};

template <typename T>
class offlineLoader
{
public:
struct iterator;

private:
    std::list< ev::packet<T> > data;
    iterator _begin;//{{nullptr, nullptr, -1, -1.0, nullptr}};
    iterator _end;//{{nullptr, nullptr, -1, -1.0, nullptr}};
    double time_sync_offset{0.0};
    int event_count{0};

    bool setIterators(typename std::list< packet<T> >::iterator same_packet)
    {
        if(data.size() == 0) return false;
        bool ret = true;

        //if same_packet is the end of data set both pointers to point to the final packet end
        //else set them to the same packet start
        if(same_packet == data.end()) 
        {
            same_packet = std::prev(data.end());
            _begin.m_ptr = same_packet->end();
            ret = false;
        } else {
            _begin.m_ptr = same_packet->begin();
        }
        _begin.packet_it = same_packet;
        _begin.final = same_packet;
        _begin._id = same_packet->id();
        _begin._timestamp = same_packet->timestamp();
        _end = _begin;
        return ret;
    }

    bool setIterators(typename std::list< packet<T> >::iterator first_packet, typename std::list< packet<T> >::iterator last_packet)
    {
        //if the first packet is finished the data invalidate the pointers.
        if(first_packet == data.end()) 
        {
            return setIterators(first_packet);
        }

        //if the last packet
        if(last_packet == data.end()) last_packet = std::prev(last_packet);

        _begin.packet_it = first_packet;
        _begin.final = last_packet;
        _begin._id = first_packet->id();
        _begin._timestamp = first_packet->timestamp();
        _begin.m_ptr = first_packet->begin();

        _end.packet_it = last_packet;
        _end.final = last_packet;
        _end._id = last_packet->id();
        _end._timestamp = last_packet->timestamp();
        _end.m_ptr = last_packet->end();
        return true;
    }

public:

    struct iterator
    {
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = std::ptrdiff_t;
        using value_type        = T;
        using pointer           = T*;
        using reference         = T&;

        inline double timestamp() {return _timestamp;}
        inline double packetID() {return _id;}

        T& operator*() const { return *m_ptr; }
        T* operator->() { return &(*m_ptr); }
        iterator& operator++()
        {
            m_ptr++;
            if(m_ptr == packet_it->end() && packet_it != final) {
                m_ptr = (++packet_it)->begin();
                _timestamp = packet_it->timestamp();
                _id = packet_it->id();
            }
            return *this;
        }

        iterator& operator++(int k)
        {
            m_ptr++;
            if(m_ptr == packet_it->end() && packet_it != final) {
                m_ptr = (++packet_it)->begin();
                _timestamp = packet_it->timestamp();
                _id = packet_it->id();
            }
            return *this;
        }

        friend bool operator== (const iterator& a, const iterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const iterator& a, const iterator& b) { return a.m_ptr != b.m_ptr; };
        friend offlineLoader;

        private:
            int _id{-1};
            double _timestamp{0.0};
            typename packet<T>::iterator m_ptr{nullptr};
            typename std::list< packet<T> >::iterator packet_it;
            typename std::list< packet<T> >::iterator final;
    };

    bool load(std::string path, double seconds = DBL_MAX)
    {
        if(seconds < 0.0) seconds = DBL_MAX;
        data.clear();

        std::ifstream reader;
        reader.open(path.c_str());
        if(!reader.is_open())
            return false;

        std::string data_line;
        while(getline(reader, data_line)) 
        {
            yarp::os::Bottle b(data_line);
            
            data.emplace_back(); ev::packet<T> &p = data.back();
            p.envelope() = {b.get(0).asInt32(), b.get(1).asFloat64()};
            p.duration(b.get(3).asInt32()*0.000001);
            p.fillFromMemory(b.get(4).asString().data(), b.get(4).asString().size());
            event_count += p.size();

            if(p.timestamp() - data.begin()->timestamp() > seconds) break;
        }

        //set both pointing to first event
        setIterators(data.begin());
        //time_sync_offset = -data.begin()->timestamp();

        return true;
    }

    void synchroniseRealtimeRead(double now)
    {
        time_sync_offset = now - data.begin()->timestamp();
    }

    bool incrementReadTill(double timestamp)
    {
        if(data.size() == 0) return false;
        timestamp -= time_sync_offset;

        //firstly set both iterators to point to the next packet if needed
        if(_end != _begin) {
            std::advance(_end.packet_it, 1);
            if(!setIterators(_end.packet_it))
                return false;
        }

        //if timestamp is greater than the next packet increment the _end packet
        while(std::next(_end.packet_it) != data.end() && std::next(_end.packet_it)->timestamp() < timestamp)
            std::advance(_end.packet_it, 1);

        //if the current packet is under timestamp, set the iterators to deliver this data
        if(_begin.packet_it->timestamp() < timestamp)
            setIterators(_begin.packet_it, _end.packet_it);

        return true;
    }

    bool windowedReadTill(double timestamp, double duration)
    {
        if(data.size() == 0) return false;
        timestamp -= time_sync_offset;

        //if timestamp is greater than the next packet increment the _end packet
        while(std::next(_end.packet_it) != data.end() && std::next(_end.packet_it)->timestamp() < timestamp)
            std::advance(_end.packet_it, 1);

        while(timestamp - _begin.packet_it->timestamp() > duration) {
            if(std::next(_begin.packet_it) == data.end()) {
                return false; //finish the dataset
            } else {
                std::advance(_begin.packet_it, 1); //increment the oldest packet
            }
            if(_begin.packet_it == _end.packet_it) break; //return atleast a single packet
        }

        setIterators(_begin.packet_it, _end.packet_it);

        return true;

    }

    iterator begin() { return _begin; }
    iterator end()   { return _end; }

    std::string getinfo() {
        std::stringstream ss;
        if(data.size() == 0)
            return "no events loaded";
        
        ss << data.size() << " packets loaded with " << event_count << " total events. "
           << "Timestamps range from " << std::fixed << std::setprecision(3) << data.begin()->timestamp() << " to " << std::prev(data.end())->timestamp();
        
        return ss.str();
    }

    double getLength() 
    {
        return std::prev(data.end())->timestamp() - data.begin()->timestamp();
    }


};

}



