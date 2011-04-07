#ifndef __UNMASK_H__
#define __UNMASK_H__

#include "synapse.h"

class Unmask
{
public:
	Unmask(int channel);
	~Unmask();
    
    void unmaskData(unsigned char* buffer,int size,int type);

private:
    int mChannel;
    int mWrapAdd;
    Synapse* objSynapse;
};

#endif
