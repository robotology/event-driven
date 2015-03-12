/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include <iCub/emorph/all.h>

class vCircle
{

private:

    //data
    //emorph::activityMat activity;

    //parameters
    int width;
    int height;
    int sRadius;

public:

    vCircle(int width = 128, int height = 128, int sRadius = 5) :
        sRadius(sRadius), width(width), height(height) {
        activity = emorph::activityMat(height, width, 500000, 20);
    }
    void localCircleEstimate(emorph::AddressEvent &event);

    //temporarily public
    emorph::activityMat activity;




};


