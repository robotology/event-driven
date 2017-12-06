/* event_driven.i */
%module event_driven
%{
#define SWIG_FILE_WITH_INIT
#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vBottle.h"
#include <yarp/os/Bottle.h>
#include <deque>
#include <vector>
#include <memory>
%}

%include "numpy.i"

%init %{
import_array();
%}

%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(unsigned int* r1, int n1),
                                                (unsigned int* r2, int n2),
                                                (unsigned int* r3, int n3),
                                                (unsigned int* r4, int n4),
                                                (unsigned int* r5, int n5)};
%apply (unsigned int* IN_ARRAY1, int DIM1) {(unsigned int* s1, int m1),
                                            (unsigned int* s2, int m2),
                                            (unsigned int* s3, int m3),
                                            (unsigned int* s4, int m4),
                                            (unsigned int* s5, int m5)};

namespace ev {


class vEvent
{
public:
    static const std::string tag;
    vEvent();
    virtual ~vEvent();

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel();
};


class AddressEvent : public vEvent
{
public:
    AddressEvent();
    AddressEvent(const vEvent &v);
    AddressEvent(const AddressEvent &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};
class FlowEvent : public AddressEvent
{
public:
    static const std::string tag;
    float vx;
    float vy;

    FlowEvent();
    FlowEvent(const vEvent &v);
    FlowEvent(const FlowEvent &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

    int getDeath() const;
};

class LabelledAE : public AddressEvent
{
public:
    static const std::string tag;
    int ID;

    LabelledAE();
    LabelledAE(const vEvent &v);
    LabelledAE(const LabelledAE &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

class GaussianAE : public LabelledAE
{
public:
    static const std::string tag;
    float sigx;
    float sigy;
    float sigxy;

    GaussianAE();
    GaussianAE(const vEvent &v);
    GaussianAE(const GaussianAE &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

class vBottle : public yarp::os::Bottle {

public:

    //constructors should not change from Bottle
    vBottle();

    //you can only modify contents by adding events and append other vBottles
    void append(vBottle &eb);

    template<class T> void append(vBottle &eb);

    //inherited
    void clear();

    bool read(yarp::os::ConnectionReader& reader);





};


%extend vBottle {
    int getSize() {
        return $self->findGroup("AE").size() / 2;
    }

    void _getData(unsigned int* r1, int n1,
                 unsigned int* r2, int n2,
		 unsigned int* r3, int n3,
                 unsigned int* r4, int n4,
                 unsigned int* r5, int n5)
    {
        auto q = $self->get<ev::vEvent>();
        if ((unsigned int)n1!= q.size()) {
                std::cerr << "The size of the provided array does not match the expected size" << std::endl;
        	return;
        }
        unsigned int i;
	for (i=0; i< q.size(); i++) {
            ev::AE* m = read_as(q[i]);

	    r1[i] = m->channel;
	    r2[i] = m->stamp;
	    r3[i] = m->x;
	    r4[i] = m->y;
	    r5[i] = m->polarity;
	}
        //if (abs(r2[q.size()-1]-r2[0]) > 10000) {
        //    std::cout << "ERR:meh " << r2[q.size()-1] << " - " << r2[0] << " = " << r2[q.size()-1]-r2[0]  << std::endl; 
        //}
    }

    void _setData(unsigned int* s1, int m1,
                 unsigned int* s2, int m2,
		 unsigned int* s3, int m3,
                 unsigned int* s4, int m4,
                 unsigned int* s5, int m5)
    {
        auto q = $self->get<ev::vEvent>();
        if ((unsigned int)m1!= q.size()) {
                std::cerr << "The size of the provided array does not match the expected size" << std::endl;
        	return;
        }
        unsigned int i;
	for (i=0; i< q.size(); i++) {
	    ev::AE* m = read_as(q[i]);
	    m->channel  = s1[i];
	    m->stamp    = s2[i];
	    m->x        = s3[i];
	    m->y        = s4[i];
	    m->polarity = s5[i];
        }
    }

}
%pythoncode %{
import numpy as np

def getData(binp):
    """ returns the channel, timestamp, x, y and polarity contained
    in the  vBottle provided by binp"""
    return np.stack(binp._getData(*[binp.getSize()]*5)).T

def setData(binp, data):
    """ writes the channel, timestamp, x, y and polarity provided
    in data to the vBottle provided by binp"""
    if (data.shape != (binp.getSize(), 5) or data.dtype != np.uint32):
        raise ValueError('argument data requires shape (512, 5) and uint32')
    binp._setData(*data.T)

#vBottle.getData = classmethod(_class_getData)
#vBottle.setData = classmethod(_class_setData)

%}

}

%inline %{
    ev::vBottle* bottleToVBottle(yarp::os::Bottle* base) {
    return static_cast<ev::vBottle*>(base);
  }
%}

