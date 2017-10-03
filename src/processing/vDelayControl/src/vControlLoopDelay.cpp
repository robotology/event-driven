#include "vControlLoopDelay.h"

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

void delayControl::initFilter(int width, int height, int nparticles, int bins,
                              bool adaptive, int nthreads, double minlikelihood,
                              double inlierThresh, double randoms)
{
    vpf.initialise(width, height, nparticles, bins, adaptive, nthreads,
                   minlikelihood, inlierThresh, randoms);

    res.height = height;
    res.width = width;
}

void delayControl::setFilterInitialState(int x, int y, int r)
{
    vpf.setSeed(x, y, r);
    vpf.resetToSeed();
}

void delayControl::initDelayControl(double gain, int maxtoproc, int positiveThreshold, int mindelay)
{
    this->gain = gain;
    this->minEvents = mindelay;
    qROI.setSize(maxtoproc);
    this->detectionThreshold = positiveThreshold;
}

bool delayControl::open(std::string name, unsigned int qlimit)
{
    inputPort.setQLimit(qlimit);
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
    int channel;

    //START HERE!!
    ev::vQueue *q = 0;
    while(!q && !isStopping()) {
        q = inputPort.getNextQ(ystamp);
    }
    if(isStopping()) return;

    channel = q->front()->getChannel();

    while(true) {

        //calculate error
        unsigned int delay = inputPort.queryDelayN();
        targetproc = minEvents + (int)(delay * gain);

        //update the ROI with enough events
        Tgetwindow = yarp::os::Time::now();
        unsigned int addEvents = 0;
        unsigned int testedEvents = 0;
        while(testedEvents < targetproc) {

            //if we ran out of events get a new queue
            if(i >= q->size()) {
                //if(inputPort.queryunprocessed() < 3) break;
                inputPort.scrapQ();
                q = 0; i = 0;
                while(!q && !isStopping()) {
                    q = inputPort.getNextQ(ystamp);
                }
                if(isStopping()) return;
            }

            auto v = is_event<AE>((*q)[i]);
            addEvents += qROI.add(v);
            //if(breakOnAdded) testedEvents = addEvents;
            //else testedEvents++;
            testedEvents++;
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

        //set our new position
        vpf.extractTargetPosition(avgx, avgy, avgr);
        double roisize = avgr * 1.4;
        qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);

        //set our new window #events
        double nw; vpf.extractTargetWindow(nw);
        //if(qROI.q.size() * 0.02 > nw) nw = 0;
        if(nw < 30) nw = 0;
        qROI.setSize(std::min(std::max(qROI.q.size() - nw, 50.0), 3000.0));

        //calculate window in time
        double tw = 0;
        if(nw >= qROI.q.size())
            yError() << "# window > queue";
        else
            tw = qROI.q.back()->stamp - qROI.q[(int)(nw+0.5)]->stamp;
        if(tw < 0) tw += vtsHelper::max_stamp;

        Tresample = yarp::os::Time::now();
        vpf.performResample();
        Tresample = yarp::os::Time::now() - Tresample;

        Tpredict = yarp::os::Time::now();
        vpf.performPrediction(std::max(addEvents / (5.0 * avgr), 0.7));
        Tpredict = yarp::os::Time::now() - Tpredict;

        //check for stagnancy
        if(vpf.maxlikelihood < detectionThreshold) {

            if(!stagnantstart) {
                stagnantstart = yarp::os::Time::now();
            } else {
                if(yarp::os::Time::now() - stagnantstart > 1.0) {
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
            ceg->setChannel(channel);
            ceg->x = avgx;
            ceg->y = avgy;
            ceg->sigx = avgr;
            ceg->sigy = tw;
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
        static double ratetime = yarp::os::Time::now();
        if(scopePort.getOutputCount()) {

            static int countscope = 0;
            static double val1 = 0;//-ev::vtsHelper::max_stamp;
            static double val2 = 0;//-ev::vtsHelper::max_stamp;
            static double val3 = 0;//-ev::vtsHelper::max_stamp;
            static double val4 = 0;//-ev::vtsHelper::max_stamp;
            static double val5 = 0;//-ev::vtsHelper::max_stamp;
            static double val6 = 0;//-ev::vtsHelper::max_stamp;
            static double val7 = 0;//-ev::vtsHelper::max_stamp;

            double ratetimedt = yarp::os::Time::now() - ratetime;
//            val1 = std::max(val1, (double)(1.0/ratetimedt));
//            val2 = std::max(val2, (double)inputPort.queryDelayN());
//            val3 = std::max(val3, inputPort.queryDelayT());
//            val4 = std::max(val4, inputPort.queryRate() / 1000.0);
//            val5 = std::max(val5, avgx);
//            val6 = std::max(val6, avgy);
//            val7 = std::max(val7, avgr);
            val1 += ratetimedt * 1e3;
            val2 += 1.0/ratetimedt;//(double)inputPort.queryDelayN();
            val3 += tw * vtsHelper::tsscaler * 1e3;//vpf.maxlikelihood;//inputPort.queryDelayT();
            val4 += inputPort.queryRate() / 1000.0;
            val5 += qROI.q.size() - nw;
            val6 += avgy;
            val7 += avgr;
            ratetime += ratetimedt;
            //val3 = val5;

            //val2 = 0;
            //val3 = 0;//std::max(val3, inputPort.queryDelayT());
            //val5 = 0;//std::max(val5, avgx);
            val6 = 0;//std::max(val6, avgy);
            val7 = 0;//std::max(val7, avgr);
            countscope++;

            double scopedt = yarp::os::Time::now() - pscopetime;
            if((scopedt > 0.05 || scopedt < 0) && countscope > 3) {
                pscopetime += scopedt;

                yarp::os::Bottle &scopedata = scopePort.prepare();
                scopedata.clear();
                scopedata.addDouble(val1/countscope);
                scopedata.addDouble(val2/countscope);
                scopedata.addDouble(val3/countscope);
                scopedata.addDouble(val4/countscope);
                scopedata.addDouble(val5/countscope);
                scopedata.addDouble(val6/countscope);
                scopedata.addDouble(val7/countscope);

                val1 = 0;//-ev::vtsHelper::max_stamp;
                val2 = 0;//-ev::vtsHelper::max_stamp;
                val3 = 0;//-ev::vtsHelper::max_stamp;
                val4 = 0;//-ev::vtsHelper::max_stamp;
                val5 = 0;//-ev::vtsHelper::max_stamp;
                val6 = 0;//-ev::vtsHelper::max_stamp;
                val7 = 0;//-ev::vtsHelper::max_stamp;

                countscope = 0;

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


            int px1 = avgx - roisize; if(px1 < 0) px1 = 0;
            int px2 = avgx + roisize; if(px2 >= res.width) px2 = res.width-1;
            int py1 = avgy - roisize; if(py1 < 0) py1 = 0;
            int py2 = avgy + roisize; if(py2 >= res.height) py2 = res.height-1;

            for(int x = px1; x <= px2; x+=2) {
                image(x, py1) = yarp::sig::PixelBgr(255, 255, 255);
                image(x, py2) = yarp::sig::PixelBgr(255, 255, 255);
            }
            for(int y = py1; y <= py2; y+=2) {
                image(px1, y) = yarp::sig::PixelBgr(255, 255, 255);
                image(px2, y) = yarp::sig::PixelBgr(255, 255, 255);
            }

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
    use_TW = false;
}

void roiq::setSize(unsigned int value)
{
    //if TW n is in clock-ticks
    //otherwise n is in # events.
    n = value;
    while(q.size() > n)
        q.pop_front();
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
    return 1;
    if(!use_TW) {
        while(q.size() > n)
            q.pop_front();
    } else {

        int dt = v->stamp - q.front()->stamp;
        if(dt < 0) dt += vtsHelper::max_stamp;
        while(dt > n) {
            q.pop_front();
            dt = v->stamp - q.front()->stamp;
            if(dt < 0) dt += vtsHelper::max_stamp;
        }
    }

    return 1;
}
