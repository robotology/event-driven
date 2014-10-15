#ifndef __UNMASK_H__
#define __UNMASK_H__

#include "synapse.h"

class Unmask
{
public:
	Unmask(int channel);
	~Unmask();
    
    void unmaskData(unsigned int* buffer,int size);
    void unmaskFile(const char *fileName);

private:
    int mChannel;
    int mWrapAdd;
    Synapse* objSynapse;
};

#endif
