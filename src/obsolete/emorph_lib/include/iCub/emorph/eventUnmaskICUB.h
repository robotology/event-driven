/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#ifndef EVENTUNMASKICUB_H
#define EVENTUNMASKICUB_H

//#define _DEBUG_

#include "eventUnmask.h"

#ifdef _DEBUG_
#include <cstdio>
#endif

namespace emorph
{
namespace eunmask
{

class eventUnmaskICUB:public emorph::eunmask::eventUnmask
{
public:
    eventUnmaskICUB(bool _save=false);
    eventUnmaskICUB(const eventUnmaskICUB&);

    ~eventUnmaskICUB();

    eventUnmaskICUB& operator=(const eventUnmaskICUB&);

	void setBuffer(char*, uint){};
	void setBuffer(emorph::ebuffer::eventBuffer&);
    void reshapeBuffer();
    int getUmaskedData(uint&, uint&, int&, uint&, uint&);
    int reset();
private:
    void saveBuffer(char*, uint);
    uint snapBuffer();

    void objcpy(const eventUnmaskICUB&);
    u32 *buffer;
    u32 *bufSnapShot;
    uint tsPacket;
    bool save;
#ifdef _DEBUG_
    FILE *dump;
#endif
};
}
}
#endif //EVENTUNMASKICUB_H
