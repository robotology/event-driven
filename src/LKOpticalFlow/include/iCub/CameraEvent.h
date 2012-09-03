/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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

#ifndef ADDRESSEVENT_H_
#define ADDRESSEVENT_H_

class CameraEvent{
    short row;
    short column;
    short polarity;
    unsigned long timeStamp;
    bool reliable;

public:
    CameraEvent(int row, int column, short polarity, unsigned long timeStamp);
    short getRowIdx();
    short getColumnIdx();
    short getPolarity();
    unsigned long getTimeStamp();
    void setReliable(bool );
    bool isReliable();
};



#endif /* ADDRESSEVENT_H_ */
