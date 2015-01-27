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

#ifndef HEADING_H_
#define HEADING_H_

#include "SuperReceptiveField.h"

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/os/Event.h>
#include <yarp/sig/Matrix.h>
#include <iCub/VelocityBuffer.h>
//#include <iCub/emorph/VelocityBuffer.h>

using namespace yarp::os;

class HeadingModule : public RFModule{

    int receptiveFieldsNum;
    SuperReceptiveField * receptiveField;

public:

    bool configure(ResourceFinder & rf);

    bool interruptModule();
    bool close();

    bool updateModule();

    virtual ~HeadingModule();

    //void makeObjMap(VelocityBuffer &);

};

#endif /* HEADING_H_ */
