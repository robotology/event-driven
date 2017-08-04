#include "vControlLoopDelay.h"

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

void delayControl::initFilter(int width, int height, int nparticles, int bins,
                              bool adaptive, int nthreads, double minlikelihood,
                              double inlierThresh, double randoms)
{
    vpf.initialise(width, height, nparticles, bins, adaptive, nthreads,
                   bins * minlikelihood, inlierThresh, randoms);
    res.height = height;
    res.width = width;
    detectionThreshold = bins * 0.35;
}

void delayControl::initDelayControl(double gain, double minvalue, int maxtoproc)
{
    this->gain = gain;
    this->minEvents = minvalue;
    qROI.setSize(maxtoproc);
}

bool delayControl::open(std::string name)
{
    if(!inputPort.open(name + "/vBottle:i"))
        return false;
    if(!outputPort.open(name + "/vBottle:o"))
        return false;
    if(!scopePort.open(name + "/scope:o"))
        return false;
    if(!debugPort.open(name + "/debug:o"))
        return false;

    return true;
}

void delayControl::onStop()
{
    inputPort.close();
    outputPort.close();
    scopePort.close();
    debugPort.close();
    inputPort.releaseDataLock();
}

void delayControl::run()
{

    double Tresample = 0;
    double Tpredict = 0;
    double Tlikelihood = 0;
    double Tgetwindow = 0;

    unsigned int targetproc = 0;
    unsigned int i = 0;
    yarp::os::Stamp ystamp;
    double stagnantstart = 0;
    bool detection = false;

    //START HERE!!
    ev::vQueue *q = 0;
    while(!q && !isStopping()) {
        q = inputPort.getNextQ(ystamp);
    }
    if(isStopping()) return;

    while(true) {

        //calculate error
        unsigned int delay = inputPort.queryDelayN();
        targetproc = minEvents + (int)(delay * gain);

        //update the ROI with enough events
        Tgetwindow = yarp::os::Time::now();
        unsigned int addEvents = 0;
        while(addEvents < targetproc) {

            //if we ran out of events get a new queue
            if(i >= q->size()) {
                inputPort.scrapQ();
                q = 0; i = 0;
                while(!q && !isStopping()) {
                    q = inputPort.getNextQ(ystamp);
                }
                if(isStopping()) return;
            }

            auto v = is_event<AE>((*q)[i]);
            if(v->channel)
                addEvents += qROI.add(v);
            i++;
        }
        Tgetwindow = yarp::os::Time::now() - Tgetwindow;

        //get the current time
        int currentstamp = 0;
        if(i >= q->size())
            currentstamp = (*q)[i-1]->stamp;
        else
            currentstamp = (*q)[i]->stamp;

        //do our update!!
        //yarp::os::Time::delay(0.005);
        Tlikelihood = yarp::os::Time::now();
        vpf.performObservation(qROI.q);
        Tlikelihood = yarp::os::Time::now() - Tlikelihood;
        vpf.extractTargetPosition(avgx, avgy, avgr);
        double roisize = avgr*1.5;
        qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);

        Tresample = yarp::os::Time::now();
        vpf.performResample();
        Tresample = yarp::os::Time::now() - Tresample;

        Tpredict = yarp::os::Time::now();
        vpf.performPrediction(0.7);
        Tpredict = yarp::os::Time::now() - Tpredict;
        //vpf.performPrediction(addEvents / (2.0 * avgr));

        //check for stagnancy
        if(vpf.maxlikelihood < detectionThreshold * 0.35) {

            if(!stagnantstart) {
                stagnantstart = yarp::os::Time::now();
            } else {
                if(yarp::os::Time::now() - stagnantstart > 3.0) {
                    vpf.resetToSeed();
                    detection = false;
                    stagnantstart = 0;
                    yInfo() << "Performing full resample";
                }
            }
        } else {
            detection = true;
            stagnantstart = 0;
        }


        //output our event
        if(outputPort.getOutputCount()) {
            auto ceg = make_event<GaussianAE>();
            ceg->stamp = currentstamp;
            ceg->setChannel(1);
            ceg->x = avgx;
            ceg->y = avgy;
            ceg->sigx = avgr;
            ceg->sigy = avgr;
            ceg->sigxy = 1.0;
            if(vpf.maxlikelihood > detectionThreshold)
                ceg->polarity = 1.0;
            else
                ceg->polarity = 0.0;

            vBottle &outputbottle = outputPort.prepare();
            outputbottle.clear();
            outputbottle.addEvent(ceg);
            outputPort.write();

        }


        //write to our scope
        static double pscopetime = yarp::os::Time::now();
        if(scopePort.getOutputCount()) {

            static double val1 = -ev::vtsHelper::max_stamp;
            static double val2 = -ev::vtsHelper::max_stamp;
            static double val3 = -ev::vtsHelper::max_stamp;
            static double val4 = -ev::vtsHelper::max_stamp;
            static double val5 = -ev::vtsHelper::max_stamp;

            val1 = std::max(val1, (double)delay);
            val2 = std::max(val2, Tgetwindow);
            val3 = std::max(val3, Tlikelihood);
            val4 = std::max(val4, Tresample);
            val5 = std::max(val5, Tpredict);

            double scopedt = yarp::os::Time::now() - pscopetime;
            if((scopedt > 0.05 || scopedt < 0)) {
                pscopetime += scopedt;

                yarp::os::Bottle &scopedata = scopePort.prepare();
                scopedata.clear();
                scopedata.addDouble(val1);
                scopedata.addDouble(val2);
                scopedata.addDouble(val3);
                scopedata.addDouble(val4);
                scopedata.addDouble(val5);

                val1 = -ev::vtsHelper::max_stamp;
                val2 = -ev::vtsHelper::max_stamp;
                val3 = -ev::vtsHelper::max_stamp;
                val4 = -ev::vtsHelper::max_stamp;
                val5 = -ev::vtsHelper::max_stamp;

                scopePort.write();
            }
        }

        //output a debug image
        static double pimagetime = yarp::os::Time::now();
        double pimagetimedt = yarp::os::Time::now() - pimagetime;
        if(debugPort.getOutputCount() && pimagetimedt > 0.04) {
            pimagetime += pimagetimedt;

            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugPort.prepare();
            image.resize(res.width, res.height);
            image.zero();

            std::vector<vParticle> indexedlist = vpf.getps();

            for(unsigned int i = 0; i < indexedlist.size(); i++) {

                int py = indexedlist[i].gety();
                int px = indexedlist[i].getx();

                if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
                image(px, py) = yarp::sig::PixelBgr(255, 255, 255);

            }
            drawEvents(image, qROI.q, currentstamp, 0, false);

            //drawcircle(image, avgx, avgy, avgr+0.5, 1);

            debugPort.write();
        }

    }

}

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/


roiq::roiq()
{
    roi.resize(4);
    n = 1000;
    roi[0] = 0; roi[1] = 1000;
    roi[2] = 0; roi[3] = 1000;
}

void roiq::setSize(unsigned int value)
{
    n = value;
}

void roiq::setROI(int xl, int xh, int yl, int yh)
{
    roi[0] = xl; roi[1] = xh;
    roi[2] = yl; roi[3] = yh;
}

int roiq::add(event<AE> &v)
{

    if(v->x < roi[0] || v->x > roi[1] || v->y < roi[2] || v->y > roi[3])
        return 0;
    q.push_back(v);
    if(q.size() > n)
        q.pop_front();
    return 1;
}
