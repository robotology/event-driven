#include "iCub/emorph/eventAtom.h"

namespace emorph
{
namespace evolume
{

eventAtom::eventAtom(unsigned int _x, unsigned int _y, short _pol, unsigned int _ts)
:addrx(_x), addry(_y), pol(_pol), ts(_ts)
{
    follower=0;
}
eventAtom::~eventAtom()
{

}

}
}
