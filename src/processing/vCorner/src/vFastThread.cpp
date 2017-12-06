#include "vFastThread.h"

using namespace ev;

vFastThread::vFastThread(unsigned int height, unsigned int width, std::string name, bool strict, double gain)
{
    std::cout << "Using FAST implementation..." << std::endl;

    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->gain = gain;

    surfaceOfR.resize(width, height);
    surfaceOnR.resize(width, height);
    surfaceOfL.resize(width, height);
    surfaceOnL.resize(width, height);
}

bool vFastThread::threadInit()
{

//    inputPort.setQLimit(1);
    if(!inputPort.open("/" + name + "/vBottle:i")) {
    std::cout << "could not open vBottleIn port " << std::endl;
        return false;
    }

    if(!outthread.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }
    if(!outthread.start())
        return false;

    std::string debugPortName = "/" + name + "/debug:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}


void vFastThread::onStop()
{
    debugPort.close();
    inputPort.close();
    inputPort.releaseDataLock();
}

void vFastThread::getCircle3(yarp::sig::ImageOf<yarp::sig::PixelInt> *cSurf, int x, int y, unsigned int (&p3)[16], int (&circle3)[16][2])
{
    for (int i = 0; i < 16; i++) {
        int xi = x + circle3[i][0];
        int yi = y + circle3[i][1];
        if(xi < 0 || yi < 0 || xi >= width || yi >= height)
            continue;

        p3[i] = (*cSurf)(xi, yi);

    }

}

void vFastThread::getCircle4(yarp::sig::ImageOf<yarp::sig::PixelInt> *cSurf, int x, int y, unsigned int (&p4)[20], int (&circle4)[20][2])
{
    for (int i = 0; i < 20; i++) {
        int xi = x + circle4[i][0];
        int yi = y + circle4[i][1];
        if(xi < 0 || yi < 0 || xi >= width || yi >= height)
            continue;

        p4[i] = (*cSurf)(xi, yi);
    }

}

void vFastThread::run()
{
//    int maxV = 50000;
    int minAcceptableDelay =  5120;
    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inputPort.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

        unsigned int delay_n = inputPort.queryDelayN();
//        double increment = ((double)delay_n)/maxV;

        double increment = gain * (delay_n - q->size()) / minAcceptableDelay;
        if(increment < 1.0)
            increment = 1.0;

        double currCount;
        int currSkip,lastSkip = 0;
        currCount = 0.0;
        currSkip = (int)currCount;

        int countProcessed = 0;
        bool firstChecked = false;
        ev::vQueue::iterator qi;
        while(currSkip < q->size())  {

            if(!firstChecked) {
                qi = q->begin();
                firstChecked = true;
            } else {
                qi = qi + (currSkip - lastSkip);
                lastSkip = currSkip;
            }

            lastSkip = currSkip;
            currCount += increment;
            currSkip = (int)currCount;

            auto ae = ev::is_event<ev::AE>(*qi);
            //AE* ae = read_as<AE>(*qi);
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

            unsigned int patch3[16];
            unsigned int patch4[20];
            getCircle3(cSurf, ae->x, ae->y, patch3, circle3);
            getCircle4(cSurf, ae->x, ae->y, patch4, circle4);

            bool isc = detectcornerfast(patch3, patch4);
            countProcessed++;

            //if it's a corner, add it to the output bottle
            if(isc) {
                auto ce = make_event<LabelledAE>(ae);
                ce->ID = 1;
                outthread.pushevent(ce, yarpstamp);
            }

        }

        static double prevtime = yarpstamp.getTime();

        if(debugPort.getOutputCount()) {
            double time = yarpstamp.getTime();

            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble(inputPort.queryRate());
            scorebottleout.addDouble(countProcessed/(time-prevtime));
            scorebottleout.addDouble((double)countProcessed/q->size());
            scorebottleout.addDouble(delay_n);
            scorebottleout.addDouble(inputPort.queryDelayT());
            debugPort.write();

            prevtime = time;
        }

        inputPort.scrapQ();

    }

}

/**********************************************************/
bool vFastThread::detectcornerfast(unsigned int patch3[16], unsigned int patch4[20])
{
    bool found_streak = false;

    for(int i = 0; i < 16; i++)
    {
        unsigned int ti = patch3[i];

        for (int streak_size = 3; streak_size <= 6; streak_size++)
        {
            if(ti < patch3[(i-1+16)%16])
                continue;

            if(patch3[(i+streak_size-1)%16] < patch3[(i+streak_size)%16])
                continue;

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
                if(ti < patch4[(i-1+20)%20])
                    continue;

                if(patch3[(i+streak_size-1)%20] < patch3[(i+streak_size)%20])
                    continue;

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
