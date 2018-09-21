/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vControlLoopDelay.h"

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

void delayControl::initFilter(int width, int height, int nparticles, int bins,
                              bool adaptive, int nthreads, double minlikelihood,
                              double inlierThresh, double randoms, double negativeBias)
{
    vpf.initialise(width, height, nparticles, bins, adaptive, nthreads,
                   minlikelihood, inlierThresh, randoms, negativeBias);

    res.height = height;
    res.width = width;
}

void delayControl::setMinRawLikelihood(double value)
{
    if(value > 0) {
        vpf.setMinLikelihood(value);
    }
}

void delayControl::setFilterInitialState(int x, int y, int r)
{
    vpf.setSeed(x, y, r);
    vpf.resetToSeed();
}

void delayControl::setMaxRawLikelihood(int value)
{
    maxRawLikelihood = value;
}

void delayControl::setNegativeBias(int value)
{
    vpf.setNegativeBias(value);
}

void delayControl::setInlierParameter(int value)
{
    vpf.setInlierParameter(value);
}

void delayControl::setMotionVariance(double value)
{
    motionVariance = value;
}

void delayControl::setTrueThreshold(double value)
{
    detectionThreshold = value * maxRawLikelihood;
}

void delayControl::setAdaptive(double value)
{
    vpf.setAdaptive(value);
}

void delayControl::setGain(double value)
{
    gain = value;
}

void delayControl::setMinToProc(int value)
{
    minEvents = value;
}

void delayControl::setResetTimeout(double value)
{
    resetTimeout = value;
}

void delayControl::performReset()
{
    vpf.resetToSeed();
}

yarp::sig::Vector delayControl::getTrackingStats()
{
    yarp::sig::Vector stats(10);

    stats[0] = 1000*inputPort.queryDelayT();
    stats[1] = 1.0/filterPeriod;
    stats[2] = targetproc;
    stats[3] = inputPort.queryRate() / 1000.0;
    stats[4] = dx;
    stats[5] = dy;
    stats[6] = dr;
    stats[7] = vpf.maxlikelihood / (double)maxRawLikelihood;
    stats[8] = cpuusage.getProcessorUsage();
    stats[9] = qROI.n;

    return stats;
}


bool delayControl::open(std::string name, unsigned int qlimit)
{
    inputPort.setQLimit(qlimit);
    if(!inputPort.open(name + "/vBottle:i"))
        return false;
    outputPort.setWriteType(GaussianAE::tag);
    if(!outputPort.open(name + "/vBottle:o"))
        return false;
//    if(!scopePort.open(name + "/scope:o"))
//        return false;
    if(!debugPort.open(name + "/debug:o"))
        return false;

    return true;
}

void delayControl::onStop()
{
    inputPort.close();
    outputPort.close();
    //scopePort.close();
    debugPort.close();
    //inputPort.releaseDataLock();
}

void delayControl::run()
{

    double Tresample = 0;
    double Tpredict = 0;
    double Tlikelihood = 0;
    double Tgetwindow = 0;

    targetproc = 0;
    unsigned int i = 0;
    yarp::os::Stamp ystamp;
    double stagnantstart = 0;
    int channel;
    qROI.setSize(50.0);

    //START HERE!!
    const vQueue *q = inputPort.read(ystamp);
    if(!q || isStopping()) return;
    vpf.extractTargetPosition(avgx, avgy, avgr);

    channel = q->front()->getChannel();

    while(true) {

        //calculate error
        double delay = inputPort.queryDelayT();
        unsigned int unprocdqs = inputPort.queryunprocessed();
        targetproc = M_PI * avgr;
        if(unprocdqs > 1 && delay > gain)
            targetproc *= (delay / gain);

        //targetproc = minEvents + (int)(delay * gain);
        //targetproc = M_PI * avgr * minEvents + (int)(delay * gain);

        //update the ROI with enough events
        Tgetwindow = yarp::os::Time::now();
        unsigned int addEvents = 0;
        unsigned int testedEvents = 0;
        while(addEvents < targetproc) {

            //if we ran out of events get a new queue
            if(i >= q->size()) {
                //if(inputPort.queryunprocessed() < 3) break;
                //inputPort.scrapQ();
                i = 0;
                q = inputPort.read(ystamp);
                if(!q || isStopping()) return;
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
        dx = avgx, dy = avgy, dr = avgr;
        vpf.extractTargetPosition(avgx, avgy, avgr);
        dx = avgx - dx; dy = avgy - dy; dr = avgr - dr;
        double roisize = avgr * 1.4;
        qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);

        //set our new window #events
        double nw; vpf.extractTargetWindow(nw);
        if(qROI.q.size() - nw > 30)
            qROI.setSize(std::max(nw, 50.0));
        if(qROI.q.size() > 3000)
            qROI.setSize(3000);

        //calculate the temporal window of the q
        double tw = qROI.q.front()->stamp - qROI.q.back()->stamp;
        if(tw < 0) tw += vtsHelper::max_stamp;

        Tresample = yarp::os::Time::now();
        vpf.performResample();
        Tresample = yarp::os::Time::now() - Tresample;

        Tpredict = yarp::os::Time::now();
        //vpf.performPrediction(std::max(addEvents / (5.0 * avgr), 0.7));
        vpf.performPrediction(motionVariance);
        Tpredict = yarp::os::Time::now() - Tpredict;

        //check for stagnancy
        if(vpf.maxlikelihood < detectionThreshold) {

            if(!stagnantstart) {
                stagnantstart = yarp::os::Time::now();
            } else {
                if(yarp::os::Time::now() - stagnantstart > resetTimeout) {
                    vpf.resetToSeed();
                    stagnantstart = 0;
                }
            }

        } else {
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

            vQueue outq; outq.push_back(ceg);
            outputPort.write(outq, ystamp);

        }

        static double prev_update_time = Tgetwindow;
        filterPeriod = Time::now() - prev_update_time;
        prev_update_time += filterPeriod;

//        //write to our scope
//        static double pscopetime = yarp::os::Time::now();
//        static double ratetime = yarp::os::Time::now();
//        if(scopePort.getOutputCount()) {

//            static int countscope = 0;
//            static double val1 = 0;
//            static double val2 = 0;
//            static double val3 = 0;
//            static double val4 = 0;
//            static double val5 = 0;
//            static double val6 = 0;
//            static double val7 = 0;
//            static double val8 = 0;
//            //static double val9 = 0;
//            static double val10 = 0;

//            double ratetimedt = yarp::os::Time::now() - ratetime;
//            val1 += inputPort.queryDelayT();
//            val2 += 1.0/ratetimedt;
//            val3 += targetproc;
//            val4 += inputPort.queryRate() / 1000.0;
//            val5 += dx;
//            val6 += dy;
//            val7 += dr;
//            val8 += vpf.maxlikelihood / (double)maxRawLikelihood;
//            //val9 += cpuusage.getProcessorUsage();
//            val10 += qROI.n;
//            ratetime += ratetimedt;
//            countscope++;

//            double scopedt = yarp::os::Time::now() - pscopetime;
//            if((scopedt > 0.1 || scopedt < 0) && countscope > 3) {
//                pscopetime += scopedt;

//                yarp::os::Bottle &scopedata = scopePort.prepare();
//                scopedata.clear();
//                scopedata.addDouble(1000*val1/countscope);
//                scopedata.addDouble(val2/countscope);
//                scopedata.addDouble(val3/countscope);
//                scopedata.addDouble(val4/countscope);
//                scopedata.addDouble(val5/countscope);
//                scopedata.addDouble(val6/countscope);
//                scopedata.addDouble(val7/countscope);
//                scopedata.addDouble(val8/countscope);
//                //scopedata.addDouble(5000 * cpuusage.getProcessorUsage());
//                scopedata.addDouble(0.0);
//                scopedata.addDouble(val10/countscope);

//                val1 = 0;//-ev::vtsHelper::max_stamp;
//                val2 = 0;//-ev::vtsHelper::max_stamp;
//                val3 = 0;//-ev::vtsHelper::max_stamp;
//                val4 = 0;//-ev::vtsHelper::max_stamp;
//                val5 = 0;//-ev::vtsHelper::max_stamp;
//                val6 = 0;//-ev::vtsHelper::max_stamp;
//                val7 = 0;//-ev::vtsHelper::max_stamp;
//                val8 = 0;
//                //val9 = 0;
//                val10 = 0;

//                countscope = 0;

//                scopePort.write();
//            }
//        }

        //output a debug image
        if(debugPort.getOutputCount()) {

            //static double prev_likelihood = vpf.maxlikelihood;
            static int NOFPANELS = 3;

            static yarp::sig::ImageOf< yarp::sig::PixelBgr> *image_ptr = 0;
            static int panelnumber = NOFPANELS;

            static double pimagetime = yarp::os::Time::now();

            //if we are in waiting state, check trigger condition
            bool trigger_capture = false;
            if(panelnumber >= NOFPANELS) {
                //trigger_capture = prev_likelihood > detectionThreshold &&
                 //       vpf.maxlikelihood <= detectionThreshold;
                trigger_capture = yarp::os::Time::now() - pimagetime > 0.1;
            }
            //prev_likelihood = vpf.maxlikelihood;

            //if we are in waiting state and
            if(trigger_capture) {
                //trigger the capture of the panels only if we aren't already
                pimagetime = yarp::os::Time::now();
                yarp::sig::ImageOf< yarp::sig::PixelBgr> &image_ref =
                        debugPort.prepare();
                image_ptr = &image_ref;
                image_ptr->resize(res.width * NOFPANELS, res.height);
                image_ptr->zero();
                panelnumber = 0;
            }

            if(panelnumber < NOFPANELS) {

                yarp::sig::ImageOf<yarp::sig::PixelBgr> &image = *image_ptr;
                int panoff = panelnumber * res.width;

                int px1 = avgx - roisize; if(px1 < 0) px1 = 0;
                int px2 = avgx + roisize; if(px2 >= res.width) px2 = res.width-1;
                int py1 = avgy - roisize; if(py1 < 0) py1 = 0;
                int py2 = avgy + roisize; if(py2 >= res.height) py2 = res.height-1;

                px1 += panoff; px2 += panoff;
                for(int x = px1; x <= px2; x+=2) {
                    image(x, py1) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                    image(x, py2) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                }
                for(int y = py1; y <= py2; y+=2) {
                    image(px1, y) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                    image(px2, y) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                }

                std::vector<vParticle> indexedlist = vpf.getps();

                for(unsigned int i = 0; i < indexedlist.size(); i++) {

                    int py = indexedlist[i].gety();
                    int px = indexedlist[i].getx();

                    if(py < 0 || py >= res.height || px < 0 || px >= res.width)
                        continue;
                    int pscale = 255 * indexedlist[i].getl() / maxRawLikelihood;
                    image(px+panoff, py) =
                            yarp::sig::PixelBgr(pscale, 255, pscale);

                }
                drawEvents(image, qROI.q, panoff);

                panelnumber++;
            }

            if(panelnumber == NOFPANELS) {
                panelnumber++;
                debugPort.write();
            }

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
        q.pop_back();
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
    q.push_front(v);
    return 1;
}
