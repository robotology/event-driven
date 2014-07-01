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

//#include "CameraEvent.h"


CameraEvent::CameraEvent(int row, int column, short polarity, unsigned long timeStamp){
    this->row  = row;
    this->column  = column;
    this->polarity = polarity;
    this->timeStamp = timeStamp;

}

short CameraEvent::getRowIdx(){
    return row;
}

short CameraEvent::getColumnIdx(){
    return column;
}

short CameraEvent::getPolarity(){
    return polarity;
}

unsigned long CameraEvent::getTimeStamp(){
    return timeStamp;
}
void CameraEvent::setReliable(bool val){
    reliable = val;
}

bool CameraEvent::isReliable(){
    return reliable;
}
