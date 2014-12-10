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
#ifndef UNMASKDVS128_HPP
#define UNMASKDVS128_HPP

#include "unmask.hpp"

//#define _DEBUG

class unmaskDVS128:public unmask
{
public:
    unmaskDVS128(uint _type=0, bool _save=false);
    unmaskDVS128(const unmaskDVS128&);
    
    ~unmaskDVS128();

    unmaskDVS128& operator=(const unmaskDVS128&);
//Internally, the raw stereo addresses presently use bit 15 to mark the right eye, i.e. 1=right, 0=left. However this assignment is also likely to change without notice.
	void setBuffer(char*, uint);
    void reshapeBuffer();
    int getUmaskedData(uint&, uint&, int&, uint&, uint&);

    int reset();
private:
    void saveBuffer(char*, uint);
    uint snapBuffer();

    void objcpy(const unmaskDVS128&);
    char *buffer;
    char *bufSnapShot;

    uint typeOfRecord;
    bool save;
};

#endif //UNMASKDVS128_HPP
