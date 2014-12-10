#include "iCub/emorph/eventSpatiotemporalVolumeStack.h"

namespace emorph
{
namespace evolume
{

//#define _DEBUG_

using namespace std;
eventSpatiotemporalVolumeStack::eventSpatiotemporalVolumeStack()
{
    maxSize=1000;
    ptr_stv = new emorph::evolume::eventSpatiotemporalVolume*[maxSize];
    sz = 0;
}

eventSpatiotemporalVolumeStack::eventSpatiotemporalVolumeStack(const eventSpatiotemporalVolumeStack& _in)
{
    for(int overStack=0; overStack<sz; overStack++)
        delete ptr_stv[overStack]; 
    delete[] ptr_stv;
    ptr_stv = new emorph::evolume::eventSpatiotemporalVolume*[maxSize];
    memcpy(ptr_stv, _in.ptr_stv, _in.maxSize*sizeof(emorph::evolume::eventSpatiotemporalVolume*));
    maxSize=_in.maxSize;
    sz = _in.sz;
}

eventSpatiotemporalVolumeStack::~eventSpatiotemporalVolumeStack()
{
    for(int pointed=0; pointed<sz; pointed++)
        delete ptr_stv[pointed];
    delete[] ptr_stv;
}

eventSpatiotemporalVolumeStack& eventSpatiotemporalVolumeStack::operator=(const eventSpatiotemporalVolumeStack& _in)
{
    if(&_in!=this)
    {
        for(int overStack=0; overStack<sz; overStack++)
            delete ptr_stv[overStack];
        delete[] ptr_stv;
        ptr_stv = new emorph::evolume::eventSpatiotemporalVolume*[maxSize];
        memcpy(ptr_stv, _in.ptr_stv, _in.maxSize*sizeof(emorph::evolume::eventSpatiotemporalVolume*));
        maxSize=_in.maxSize;
        sz = _in.sz;
    }
    return *this;
}

int eventSpatiotemporalVolumeStack::upper_bound(unsigned int& _ts, unsigned int& _tmpSz)
{
#ifdef _DEBUG_
    cout    << "eventSpatiotemporalVolumeStack::upper_bound(...)" << endl
            << "\tsize of the stack: " << sz << endl;
#endif
    if(sz)
        for(int it_=sz-1; it_>=0; it_--)
        {
#ifdef _DEBUG_
            cout << "\t\tInside for: it_: " << it_ << "/" << sz << endl;
#endif
            if(ptr_stv[it_]->get_head()->get_ts()+_tmpSz<=_ts)
                return it_;
        }
    return -1;
}

void eventSpatiotemporalVolumeStack::push(unsigned int& _x, unsigned int& _y, int& _p, unsigned int& _t)
{   
#ifdef _DEBUG_
    cout << "eventSpatiotemporalVolumeStack::push(...)" << endl; 
#endif
    if( sz>=maxSize )
    {
#ifdef _DEBUG_
        cout << "Number of elements in the stack: " << sz << ", size of the stack: " << maxSize << endl;
#endif
        emorph::evolume::eventSpatiotemporalVolume** cpy_ptr_stv;
#ifdef _DEBUG_
        cout << "Allocate temporary memory..." << endl;
#endif
        try{
            cpy_ptr_stv=new emorph::evolume::eventSpatiotemporalVolume*[maxSize];
        }
        catch(bad_alloc& ba){
            cerr << "bad_alloc caught: " << ba.what() << endl;
        }
#ifdef _DEBUG_
        cout << "Copy the stvs before increasing the stack" << endl;
#endif
        memcpy(cpy_ptr_stv, ptr_stv, maxSize*sizeof(emorph::evolume::eventSpatiotemporalVolume*));
#ifdef _DEBUG_
        cout << "Delete the old stack" << endl;
#endif
        delete[] ptr_stv;
#ifdef _DEBUG_
        cout << "Allocate memory for the stack" << endl;
#endif
        try{
            ptr_stv=new emorph::evolume::eventSpatiotemporalVolume*[2*maxSize];
        }catch(bad_alloc& ba){
            cerr << "bad_alloc caught: " << ba.what() << endl;
        }
#ifdef _DEBUG_
        cout << "Copy the stack in the new allocated segment, size of the copy: " << maxSize*sizeof(emorph::evolume::eventSpatiotemporalVolume*)  << endl;
#endif
        memcpy(ptr_stv, cpy_ptr_stv, maxSize*sizeof(emorph::evolume::eventSpatiotemporalVolume*));
#ifdef _DEBUG_
        cout << "Delete the temporary allocation" << endl;
#endif
        delete[] cpy_ptr_stv;
#ifdef _DEBUG_
        cout << "Increase the size of the stack" << endl;
#endif
        maxSize=2*maxSize;
    }
#ifdef _DEBUG_
    cout << "\tAdd a new stv..." << endl; 
#endif
//    cout << "Create a new stv in the stack" << endl;
    ptr_stv[sz]=new emorph::evolume::eventSpatiotemporalVolume(_x, _y, _p, _t);
//    cout << "Increase the interator" << endl;
    ++sz;
}

void eventSpatiotemporalVolumeStack::pop(unsigned int& _it)
{
    if(_it<sz)
    {
        delete ptr_stv[_it];
        memcpy(ptr_stv[_it], ptr_stv[_it+1], (sz-(_it+1))*sizeof(emorph::evolume::eventSpatiotemporalVolume*));
        --sz;
    }
}

unsigned int eventSpatiotemporalVolumeStack::size()
{
    return sz;
}

emorph::evolume::eventSpatiotemporalVolume* eventSpatiotemporalVolumeStack::at(unsigned int _it)
{
    if(_it<sz)
        return ptr_stv[_it];
    else
        return 0;
}

unsigned int eventSpatiotemporalVolumeStack::reset()
{
    for(int pointed=0; pointed<sz; pointed++)
        delete ptr_stv[pointed];
    delete[] ptr_stv;

    maxSize=1000;
    ptr_stv = new emorph::evolume::eventSpatiotemporalVolume*[maxSize];
    sz = 0;
    return 1;
}

}
}
