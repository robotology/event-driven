#ifndef SPATIOTEMPORALVOLUMESTACK_HPP
#define SPATIOTEMPORALVOLUMESTACK_HPP

#include <iostream>
#include <cstring>

#include "iCub/emorph/eventSpatiotemporalVolume.h"
//#include "gsl_vector.h"

//#define _DEBUG_
namespace emorph
{
namespace evolume
{

class eventSpatiotemporalVolumeStack
{
public:
    eventSpatiotemporalVolumeStack();
    eventSpatiotemporalVolumeStack(const eventSpatiotemporalVolumeStack&);
    ~eventSpatiotemporalVolumeStack();
    eventSpatiotemporalVolumeStack& operator=(const eventSpatiotemporalVolumeStack&);
    /**
    *   int upper_bound(int&);
    *   @param[in]  _ts, the current timestamp
    *   @param[in]  _tmpSz, the temporal accumulation window
    *   @return     the interator on the higher element in the stack for whose
    *               timestamp is less or egale to _ts
    */
    int upper_bound(unsigned int&, unsigned int& _tmpSz);
    /**
    *   void push(unsigned int&, unsigned int&, int&, unsigned int&)
    *   @param[in]  _x, x address of the current event
    *   @param[in]  _y, y address of the current event
    *   @param[in]  _p, polarity of the current event
    *   @param[in]  _t, timestamp of the current event
    *   @return     N/A
    */
    void push(unsigned int&, unsigned int&, int&, unsigned int&);
    /**
    *   void pop(unsigned int&)
    *   @param[in]  _it, iterator to the volume to pop from the stack
        @return     N/A
    */
    void pop(unsigned int&);
    /**
    *   unsigned int size()
    *   @return     size of the stack
    */
    unsigned int size();
    /**
    *   spatiotemporalVolume* at(unsigned int)
    *   @param[in]  _it, iterator to the volume to return
    *   @return     the pointer of the volume at _it, NULL otherwise
    */
    emorph::evolume::eventSpatiotemporalVolume* at(unsigned int);

    /**
    *   unsigned int reset()
    *   @return 1 in success, 0 otherwise
    */
    unsigned int reset();
private:
    emorph::evolume::eventSpatiotemporalVolume** ptr_stv;
    unsigned int sz;
    unsigned int maxSize;
};

}
}
#endif //SPATIOTEMPORALVOLUMESTACK_HPP
