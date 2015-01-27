#include "iCub/emorph/eventSpatiotemporalVolume.h"

namespace emorph
{
namespace evolume
{

eventSpatiotemporalVolume::eventSpatiotemporalVolume()
{
    evtHead=0;
    evtLast=0;
    evtPointed=0;
    numberOfPosEvent=0;
    numberOfNegEvent=0;
/*
    weight_pos=0;
    weight_neg=0;

    repr_pos=0;
    repr_neg=0;
*/
}
eventSpatiotemporalVolume::eventSpatiotemporalVolume(unsigned int _x, unsigned int _y, short _pol, unsigned int _ts)
{
//    cout << "Call stv::stv(...)" << endl;
    evtHead=new emorph::evolume::eventAtom(_x, _y, _pol, _ts);
    evtPointed=evtHead;
    evtLast=evtHead;

    if(_pol>0)
    {
        numberOfPosEvent=1;
        numberOfNegEvent=0;
    }
    else{
        numberOfPosEvent=0;
        numberOfNegEvent=1;
    }
/*
    weight_pos=0;
    weight_neg=0;

    repr_pos=0;
    repr_neg=0;
*/
}

eventSpatiotemporalVolume::~eventSpatiotemporalVolume()
{
//    cout << "stv destructor called..." << endl;
//    cout << "\tNumber of event: " << get_numberOfevent() << endl;
    this->rewind();
//    unsigned int deleted=0;
    emorph::evolume::eventAtom* _tmp;
    while(get_pointed()->get_follower()!=0)
    {
        _tmp=get_pointed();
        forward();
        delete _tmp;
//        deleted++;
    }
    delete get_pointed();
/*
    if(weight_pos!=0){
        delete[] weight_pos;
        delete[] repr_pos;
    }
    if(weight_neg!=0){
        delete[] weight_neg;
        delete[] repr_neg;
    }
*/
//    deleted++;
//    cout << "\tNumber of event deleted: " << deleted << endl;
}
emorph::evolume::eventAtom* eventSpatiotemporalVolume::create_event(unsigned int _x, unsigned int _y, short _pol, unsigned int _ts)
{
    return new eventAtom(_x, _y, _pol, _ts);
}

void eventSpatiotemporalVolume::add_evt(emorph::evolume::eventAtom* _evt)
{
    if(this->get_numberOfevent()==0)
    {
        evtHead=_evt;
        evtLast=_evt;
        evtPointed=_evt;
    }
    else
    {
        evtLast->set_follower(_evt);
        evtLast=_evt;
        evtLast->set_follower(0);
    }
    if(_evt->get_pol()>0)
        numberOfPosEvent++;
    else
        numberOfNegEvent++;
}

void eventSpatiotemporalVolume::add_evt(unsigned int _x, unsigned int _y, short _pol, unsigned int _ts)
{
    add_evt(create_event(_x, _y, _pol, _ts));
}

bool eventSpatiotemporalVolume::forward()
{
    if(iterator<get_numberOfevent())
    {
        evtPointed=evtPointed->get_follower();
        if(evtPointed==0)
        {
            evtPointed=evtHead;
            return false;
        }
        else
        {
            iterator++;
            return true;
        }
    }
    else
    {
        evtPointed=evtHead;
        return false;
    }
}
/*
void eventSpatiotemporalVolume::set_weight(gsl_vector* _w, short _pol)
{
    if(_pol>0)
        weight_pos=_w;
    else
        weight_neg=_w;
}

void eventSpatiotemporalVolume::set_repr(gsl_matrix* _r, short _pol)
{
    if(_pol>0)
        repr_pos=_r;
    else
        repr_neg=_r;

}

gsl_vector* eventSpatiotemporalVolume::get_weight(short _pol)
{
    if(_pol>0)
        return weight_pos;
    else
        return weight_neg;
}
gsl_matrix* eventSpatiotemporalVolume::get_repr(short _pol)
{
    if(_pol>0)
        return repr_pos;
    else
        return repr_neg;
}*/

}
}
