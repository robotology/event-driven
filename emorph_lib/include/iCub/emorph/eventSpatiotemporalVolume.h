#ifndef EVENTSPATIOTEMPORALVOLUME_H
#define EVENTSPATIOTEMPORALVOLUME_H

#include "iCub/emorph/eventAtom.h"

//#include "gsl_matrix.h"
//#include "gsl_vector.h"

namespace emorph
{
namespace evolume
{

class eventSpatiotemporalVolume
{
public:
    eventSpatiotemporalVolume();
    eventSpatiotemporalVolume(unsigned int, unsigned int, short, unsigned int);
    ~eventSpatiotemporalVolume();
    void add_evt(emorph::evolume::eventAtom*);
    void add_evt(unsigned int, unsigned int, short, unsigned int);
    bool forward();

    inline void rewind(){evtPointed=evtHead; iterator=1;};

//    void set_weight(gsl_vector*, short);
//    void set_repr(gsl_matrix*, short);

    inline emorph::evolume::eventAtom* get_head(){return evtHead;};
    inline emorph::evolume::eventAtom* get_pointed(){return evtPointed;};
//    gsl_vector* get_weight(short);
//    gsl_matrix* get_repr(short);

    inline unsigned int get_numberOfevent(){return (numberOfPosEvent+numberOfNegEvent);};

private:
    emorph::evolume::eventAtom* create_event(unsigned int, unsigned int, short, unsigned int);

    unsigned int numberOfPosEvent;
    unsigned int numberOfNegEvent;
    unsigned int iterator;

    emorph::evolume::eventAtom* evtHead;
    emorph::evolume::eventAtom* evtPointed;
    emorph::evolume::eventAtom* evtLast;
/*
    gsl_vector* weight_pos;
    gsl_vector* weight_neg;
    gsl_matrix* repr_pos;
    gsl_matrix* repr_neg;
*/
};

}
}
#endif //EVENTSPATIOTEMPORALVOLUME_H
