/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco based on Elias Mueggler's implementation
 * email:  valentina.vasco@iit.it
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

#include "vFastCallback.h"

using namespace ev;

vFastCallback::vFastCallback(int height, int width)
{
    std::cout << "Using FAST implementation..." << std::endl;

    this->height = height;
    this->width = width;

    surfaceOfR.resize(width, height);
    surfaceOnR.resize(width, height);
    surfaceOfL.resize(width, height);
    surfaceOnL.resize(width, height);

    this->tout = 0;

}
/**********************************************************/
bool vFastCallback::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<ev::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    std::string debugPortName = "/" + moduleName + "/debug:o";
    bool check3 = debugPort.open(debugPortName);

    return check1 && check2 && check3;

}

/**********************************************************/
void vFastCallback::close()
{
    //close ports
    debugPort.close();
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

}

/**********************************************************/
void vFastCallback::interrupt()
{
    //pass on the interrupt call to everything needed
    debugPort.interrupt();
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();

}

/**********************************************************/
void vFastCallback::onRead(ev::vBottle &bot)
{
    ev::vBottle fillerbottle;
    bool isc = false;

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<AE>();
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto ae = is_event<AE>(*qi);
        yarp::sig::ImageOf< yarp::sig::PixelInt > *cSurf;
        if(ae->getChannel()) {
            if(ae->polarity)
                cSurf = &surfaceOfR;
            else
                cSurf = &surfaceOnR;
        } else {
            if(ae->polarity)
                cSurf = &surfaceOfL;
            else
                cSurf = &surfaceOnL;
        }

        //unwrap stamp and add the event to the surface
        (*cSurf)(ae->x, ae->y) = unwrapper(ae->stamp);

        //get events on circles
        unsigned int patch3[16];
        unsigned int patch4[20];
        getCircle3(cSurf, ae->x, ae->y, patch3, circle3);
        getCircle4(cSurf, ae->x, ae->y, patch4, circle4);

        //detect corner
        isc = detectcornerfast(patch3, patch4);

        //if it's a corner, add it to the output bottle
        if(isc) {
            auto ce = make_event<LabelledAE>(ae);
            ce->ID = 1;
            fillerbottle.addEvent(ce);
        }

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            debugPort.write();
        }
    }

    if( (yarp::os::Time::now() - tout) > 0.001 && fillerbottle.size() ) {
        yarp::os::Stamp st;
        this->getEnvelope(st);
        outPort.setEnvelope(st);
        ev::vBottle &eventsout = outPort.prepare();
        eventsout.clear();
        eventsout = fillerbottle;
        outPort.write(strictness);
        fillerbottle.clear();
        tout = yarp::os::Time::now();
    }

}

/**********************************************************/
void vFastCallback::getCircle3(yarp::sig::ImageOf<yarp::sig::PixelInt> *cSurf, int x, int y, unsigned int (&p3)[16], int (&circle3)[16][2])
{
    for (int i = 0; i < 16; i++) {
        int xi = x + circle3[i][0];
        int yi = y + circle3[i][1];
        if(xi < 0 || yi < 0 || xi >= width || yi >= height)
            continue;

        p3[i] = (*cSurf)(xi, yi);

    }

}

/**********************************************************/
void vFastCallback::getCircle4(yarp::sig::ImageOf<yarp::sig::PixelInt> *cSurf, int x, int y, unsigned int (&p4)[20], int (&circle4)[20][2])
{
    for (int i = 0; i < 20; i++) {
        int xi = x + circle4[i][0];
        int yi = y + circle4[i][1];
        if(xi < 0 || yi < 0 || xi >= width || yi >= height)
            continue;

        p4[i] = (*cSurf)(xi, yi);
    }

}

/**********************************************************/
bool vFastCallback::detectcornerfast(unsigned int patch3[16], unsigned int patch4[20])
{
    bool found_streak = false;

    for(int i = 0; i < 16; i++)
    {
        unsigned int ti = patch3[i];

        for (int streak_size = 3; streak_size <= 6; streak_size++)
        {
            //find the minimum timestamp in the current arc
            unsigned int min_t = ti;
            for (int j = 1; j < streak_size; j++)
            {
                int curri = (i+j)%16;
                unsigned int tj = patch3[curri];

                if (tj < min_t)
                    min_t = tj;
            }

            bool did_break = false;
            for (int j = streak_size; j < 16; j++)
            {
                int curri = (i+j)%16;
                unsigned int tj = patch3[curri];

                if (tj >= min_t)
                {
                    did_break = true;
                    break;
                }
            }

            if(did_break == false)
            {
                found_streak = true;
                break;
            }

        }
        if (found_streak)
        {
            break;
        }
    }

    if (found_streak)
    {
        found_streak = false;
        for (int i = 0; i < 20; i++)
        {
            unsigned int ti = patch4[i];

            for (int streak_size = 4; streak_size<= 8; streak_size++)
            {

                unsigned int min_t = ti;
                for (int j = 1; j < streak_size; j++)
                {
                    int curri = (i+j)%20;
                    unsigned int tj = patch4[curri];

                    if (tj < min_t)
                        min_t = tj;
                }

                bool did_break = false;
                for (int j = streak_size; j < 20; j++)
                {

                    int curri = (i+j)%20;
                    unsigned int tj = patch4[curri];

                    if (tj >= min_t)
                    {
                        did_break = true;
                        break;
                    }
                }

                if (!did_break)
                {
                    found_streak = true;
                    break;
                }
            }
            if (found_streak)
            {
                break;
            }
        }
    }

    return found_streak;

}

//empty line to make gcc happy

