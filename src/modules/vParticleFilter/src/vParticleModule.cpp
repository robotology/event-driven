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

#include "vParticleModule.h"

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, eventdriven::vQueue &q, double tw = 0) {

    int tnow = q.front()->getStamp(); //only valid for certain q's!!

    for(unsigned int i = 0; i < q.size(); i++) {
        if(tw) {
            double dt = tnow - q[i]->getStamp();
            if(dt < 0) dt += eventdriven::vtsHelper::maxStamp();
            if(dt > tw) break;
        }
        eventdriven::AddressEvent *v = q[i]->getUnsafe<eventdriven::AddressEvent>();
        image(v->getY(), 127 - v->getX()) = yarp::sig::PixelBgr(0, 255, 0);
    }
}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id = 0)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8) continue;
            int px = cx + x; int py = cy + y;
            if(py < 0 || py > 127 || px < 0 || px > 127) continue;
            switch(id) {
            case(0): //green
                image(py, 127 - px) = yarp::sig::PixelBgr(0, 255, 0);
                break;
            case(1): //blue
                image(py, 127 - px) = yarp::sig::PixelBgr(0, 0, 255);
                break;
            case(2): //red
                image(py, 127 - px) = yarp::sig::PixelBgr(255, 0, 0);
                break;
            default:
                image(py, 127 - px) = yarp::sig::PixelBgr(255, 255, 0);
                break;

            }

        }
    }



}

/*////////////////////////////////////////////////////////////////////////////*/
//vParticleModule
/*////////////////////////////////////////////////////////////////////////////*/
bool vParticleModule::configure(yarp::os::ResourceFinder &rf)
{
    //administrative options
    setName((rf.check("name", yarp::os::Value("vParticleFilter")).asString()).c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    int nParticles = rf.check("particles", yarp::os::Value(100)).asInt();
    double nRandResample =
            1.0 + rf.check("randoms", yarp::os::Value(2)).asDouble() / 100.0;
    bool adaptivesampling = rf.check("adaptive", yarp::os::Value(false)).asBool();
    double minlikelihood = rf.check("obsmin", yarp::os::Value(1.0)).asDouble();
    double inlierParameter = rf.check("inlierPar", yarp::os::Value(1.5)).asDouble();
    double outlierParameter = rf.check("outlierPar", yarp::os::Value(3.0)).asDouble();
    bool realtime = rf.check("userealtime", yarp::os::Value(true)).asBool();

    if(!realtime) {
        particleThread = 0;
        /* USE FULL PROCESS IN CALLBACK */
        particleCallback = new vParticleReader(
                    rf.check("width", yarp::os::Value(128)).asInt(),
                    rf.check("height", yarp::os::Value(128)).asInt());

        //open the ports
        if(!particleCallback->open(getName(), strict)) {
            std::cerr << "Could not open required ports" << std::endl;
            return false;
        }
    } else {
        particleCallback = 0;
        /* USE REAL-TIME THREAD */
        particleThread = new particleProcessor(
                    rf.check("width", yarp::os::Value(128)).asInt(),
                    rf.check("height", yarp::os::Value(128)).asInt(),
                    this->getName(), strict);

        if(!particleThread->start())
            return false;
    }


    return true;
}

/******************************************************************************/
bool vParticleModule::interruptModule()
{
    if(particleCallback) particleCallback->interrupt();
    if(particleThread) particleThread->stop();
    std::cout << "Interrupt Successful" << std::endl;
    return true;
}

/******************************************************************************/
bool vParticleModule::close()
{
    if(particleCallback) particleCallback->close();
    std::cout << "Close Successful" << std::endl;
    return true;
}

/******************************************************************************/
bool vParticleModule::updateModule()
{
    return true;
}

/******************************************************************************/
double vParticleModule::getPeriod()
{
    return 1;

}
/*////////////////////////////////////////////////////////////////////////////*/
//surfacehandler
/*////////////////////////////////////////////////////////////////////////////*/
vSurfaceHandler::vSurfaceHandler(unsigned int width, unsigned int height)
{

    res.width = width; res.height = height;
    surfaceLeft = eventdriven::temporalSurface(width, height);
    surfaceRight = eventdriven::temporalSurface(width, height);
    strict = false;
    pstamp = yarp::os::Stamp();
    ptime = yarp::os::Time::now();
    condTime = 0;
    tw = 0;
    eventrate = 0;
    bottletime = yarp::os::Time::now();
    eventsQueried = false;
    waitsignal.wait(); //lock the resource to start with

}

bool vSurfaceHandler::open(const std::string &name, bool strictness)
{
    if(strictness) {
        this->strict = true;
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();
    if(!yarp::os::BufferedPort<eventdriven::vBottle>::open("/" + name + "/vBottle:i"))
        return false;

    return true;
}

void vSurfaceHandler::close()
{
    yarp::os::BufferedPort<eventdriven::vBottle>::close();
}

void vSurfaceHandler::interrupt()
{
    yarp::os::BufferedPort<eventdriven::vBottle>::interrupt();
}

void vSurfaceHandler::onRead(eventdriven::vBottle &inputBottle)
{
    yarp::os::Stamp st;
    getEnvelope(st);
    if(st.getCount() != pstamp.getCount() +1 && pstamp.isValid()) {
        std::cout << "Lost Bottle" << std::endl;
    }
    pstamp = st;


    //create event queue
    eventdriven::vQueue q = inputBottle.get<eventdriven::AddressEvent>();
    q.sort(true);

    for(eventdriven::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        mutexsignal.lock();
        if((*qi)->getChannel() == 0)
            surfaceLeft.addEvent(**qi);
        else if((*qi)->getChannel() == 1)
            surfaceRight.addEvent(**qi);
        else
            std::cout << "Unknown channel" << std::endl;
        mutexsignal.unlock();


        //tnow = unwrap((*qi)->getStamp());


//        if(eventsQueried /*&& tnow > condTime*/) {
//            //queriedQ = surfaceLeft.getSurf_Tlim(tw);
//            eventsQueried = false;
//            waitsignal.post();
//        }


    }

//    double ctime = yarp::os::Time::now();
//    eventrate = 0.5 * (eventrate + q.size() / (ctime - bottletime));
//    bottletime = ctime;


   // if(q.size() > 1)
     //   std::cout << (double)q.size() * 7812500 / (q.back()->getStamp() - q.front()->getStamp()) <<std::endl;



}

eventdriven::vQueue vSurfaceHandler::queryEventList(std::vector<vParticle> &ps)
{

    mutexsignal.lock();
    std::vector<vParticle>::iterator i;
    eventdriven::vQueue temp;
    for(i = ps.begin(); i < ps.end(); i++) {
        temp = surfaceLeft.getSurf_Tlim(i->gettw(), i->getx(), i->gety(), i->getr() + 2);
    }
    mutexsignal.unlock();

    return temp;

}

void vSurfaceHandler::queryEvents(eventdriven::vQueue &fillq, unsigned int temporalwindow)
{
    mutexsignal.lock();
    //surfaceLeft.getSurfSorted(fillq);
    fillq = surfaceLeft.getSurf_Tlim(temporalwindow);
    mutexsignal.unlock();
}

eventdriven::vQueue vSurfaceHandler::queryEvents(unsigned long int conditionTime, unsigned int temporalWindow)
{



    condTime = conditionTime;
    tw = temporalWindow;

//    if(false) { //tnow < condTime
//        eventsQueried = true;
//        waitsignal.wait();
//    }
    mutexsignal.lock();
    //eventdriven::vQueue temp = surfaceLeft.getSurf_Tlim(tw);
    eventdriven::vQueue temp;
    surfaceLeft.getSurfSorted(temp);
    mutexsignal.unlock();



    return temp;




//    condTime = conditionTime;
//    tw = temporalWindow;
//    eventsQueried = true;
//    waitsignal.wait();
//    return queriedQ;
}
/*////////////////////////////////////////////////////////////////////////////*/
//particleprocessor (real-time)
/*////////////////////////////////////////////////////////////////////////////*/
particleProcessor::particleProcessor(unsigned int height, unsigned int weight, std::string name, bool strict)
{
    res.height = height;
    res.width  = weight;
    this->name = name;
    this->strict = strict;

    avgx = 0;
    avgy = 0;
    avgr = 0;
    avgtw = 0;
    maxtw = 0;
    maxlikelihood = 1;
    nparticles = 50;
    rate = 0;
    nThreads = 7;
    pwsumsq = 0;

    for(int i = 0; i < nThreads; i++) {
        int pStart = i * (nparticles / nThreads);
        int pEnd = (i+1) * (nparticles / nThreads);
        if(i == nThreads - 1)
            pEnd = nparticles;

        std::cout << pStart << "->" << pEnd-1 << std::endl;
        computeThreads.push_back(new vPartObsThread(pStart, pEnd));
    }

}
bool particleProcessor::threadInit()
{
    std::cout << "Initialising thread" << std::endl;

    ptime = yarp::os::Time::now();

    if(!debugOut.open("/" + name + "/debug:o")) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    if(!scopeOut.open("/" + name + "/scope:o")) {
        std::cout << "could not open scope port" << std::endl;
        return false;
    }
    if(!vBottleOut.open("/" + name + "/vBottle:o")) {
        std::cout << "coult not open vBottleOut port" << std::endl;
        return false;
    }

    if(!eventhandler.open(name, strict)) {
        std::cout << "Could not open eventhandler ports" << std::endl;
        return false;
    }

    std::cout << "Port open - querying initialisation events" << std::endl;
//    eventdriven::vQueue stw;
//    while(!stw.size()) {
//           yarp::os::Time::delay(0.01);
//           stw = eventhandler.queryEvents(0, 1);
//    }

//    unsigned int tnow = unwrap(stw.back()->getStamp());


    //initialise the particles
    vParticle p;
    p.setRate(rate);

    for(int i = 0; i < nparticles; i++) {
        p.setid(i);
        p.resample(1.0/nparticles, 0);
        p.initWeight(1.0/nparticles);
        maxtw = std::max(maxtw, p.gettw());
        indexedlist.push_back(p);
    }

    std::cout << "Thread initialised" << std::endl;
    return true;

}

void particleProcessor::run()
{
    std::cout << "Thread starting" << std::endl;

    eventdriven::vQueue stw;
    //double maxlikelihood;
    while(!isStopping()) {

        //std::cout << "GETTING EVENTS" <<std::endl;
        ptime = yarp::os::Time::now();
        eventhandler.queryEvents(stw, maxtw);
        //eventdriven::vQueue stw = eventhandler.queryEvents(unwrap.currentTime() + rate, eventdriven::vtsHelper::maxStamp() * 0.5);
        //eventdriven::vQueue stw = eventhandler.queryEventList(indexedlist);
        double Tget = (yarp::os::Time::now() - ptime)*1000.0;
        //std::cout << "size: " << stw.size() << "tw: " << maxtw << std::endl;
        //continue;
        //std::cout << "GOTTEN" <<std::endl;

        ptime = yarp::os::Time::now();


        if(!stw.size()) continue;

        unsigned long int t = unwrap(stw.front()->getStamp());

        std::vector<vParticle> indexedSnap = indexedlist;

        //resampling
        if(pwsumsq * nparticles > 2.0) {
            for(int i = 0; i < nparticles; i++) {
                //if(indexedlist[i].getw() > (0.1 / nparticles)) continue;
                double rn = 1.02 * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].resample(1.0/nparticles, t);
                else {
                    double accum = 0.0; int j = 0;
                    for(j = 0; j < nparticles; j++) {
                        accum += indexedSnap[j].getw();
                        if(accum > rn) break;
                    }
                    indexedlist[i].resample(indexedSnap[j], 1.0/nparticles, t);
                }
            }
        }

        double Tresample = (yarp::os::Time::now() - ptime)*1000.0;
        ptime = yarp::os::Time::now();

        //prediction
        maxtw = 0; //also calculate maxtw for next processing step
        for(int i = 0; i < nparticles; i++) {
            if(indexedlist[i].predict(t)) {
                indexedlist[i].resample(1.0/nparticles, t);
            }
            if(indexedlist[i].gettw() > maxtw)
                maxtw = indexedlist[i].gettw();
        }

        double Tpredict = (yarp::os::Time::now() - ptime)*1000.0;
        ptime = yarp::os::Time::now();

        //likelihood observation
        std::vector<int> deltats; deltats.resize(stw.size());
        double stampnow = stw.front()->getStamp();
        for(unsigned int i = 0; i < stw.size(); i++) {
            double dt = stampnow - stw[i]->getStamp();
            if(dt < 0)
                dt += eventdriven::vtsHelper::maxStamp();
            deltats[i] = dt;
        }

        //START SINGLE-THREAD
        //thread parameter -> index range
        //pass -> deltats, indexedlist, stw

//        for(int i = 0; i < nparticles; i++) {
//            indexedlist[i].initLikelihood();
//        }

//        for(int j = 0; j < nparticles; j++) {
//            for(unsigned int i = 0; i < stw.size(); i++) {
//                if(deltats[i] < indexedlist[j].gettw()) {
//                    eventdriven::AddressEvent *v = stw[i]->getUnsafe<eventdriven::AddressEvent>();
//                    indexedlist[j].incrementalLikelihood(v->getX(), v->getY(), deltats[i]);
//                } else {
//                    break;
//                }
//            }
//        }

//        maxlikelihood = 0;
//        double normval = 0.0;
//        for(int i = 0; i < nparticles; i++) {
//            indexedlist[i].concludeLikelihood();
//            normval += indexedlist[i].getw();
//            maxlikelihood = std::max(maxlikelihood, indexedlist[i].getl());
//        }

        //return normval for those events
        //END SINGLE-THREAD

        //START MULTI-THREAD
        for(int k = 0; k < nThreads; k++) {
            computeThreads[k]->setDataSources(&indexedlist, &deltats, &stw);
            computeThreads[k]->start();
        }

        double normval = 0.0;
        for(int k = 0; k < nThreads; k++) {
            computeThreads[k]->join();
            normval += computeThreads[k]->getNormVal();
        }
        //END MULTI-THREAD

        //normalisation
        pwsumsq = 0;
        vParticle pmax = indexedlist[0];
        for(int i = 0; i < nparticles; i ++) {
            indexedlist[i].updateWeightSync(normval);
            pwsumsq += pow(indexedlist[i].getw(), 2.0);
            if(indexedlist[i].getw() > pmax.getw())
                pmax = indexedlist[i];
        }

        double Tobs = (yarp::os::Time::now() - ptime)*1000.0;
        ptime = yarp::os::Time::now();

        //extract target position
        avgx = 0;
        avgy = 0;
        avgr = 0;
        avgtw = 0;

        for(int i = 0; i < nparticles; i ++) {
            avgx += indexedlist[i].getx() * indexedlist[i].getw();
            avgy += indexedlist[i].gety() * indexedlist[i].getw();
            avgr += indexedlist[i].getr() * indexedlist[i].getw();
            avgtw += indexedlist[i].gettw() * indexedlist[i].getw();
        }
        //std::cout << "CALCULATED AVERAGE" <<std::endl;

//        particleVariance = 0;
//        for(int i = 0; i < nparticles; i ++) {
//            //particleVariance = indexedlist[i].getw() *
//            particleVariance =
//                    (pow(avgx - indexedlist[i].getx(), 2.0) + pow(avgy - indexedlist[i].gety(), 2.0));
//        }
//        particleVariance /= nparticles;


        if(vBottleOut.getOutputCount()) {
            eventdriven::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
            eventdriven::ClusterEventGauss ceg;
            ceg.setStamp(stw.front()->getStamp());
            ceg.setChannel(1);
            ceg.setID(0);
            ceg.setNumAE(0);
            ceg.setPolarity(0);
            ceg.setXCog(avgx);
            ceg.setYCog(avgy);
            ceg.setXSigma2(avgr);
            ceg.setYSigma2(1);
            ceg.setXVel(0);
            ceg.setYVel(0);
            ceg.setXYSigma(0);

            eventsout.addEvent(ceg);
            vBottleOut.write();
        }

        //static int i = 0;
        if(debugOut.getOutputCount()) {

            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
            image.resize(res.width, res.height);
            image.zero();



            for(unsigned int i = 0; i < indexedlist.size(); i++) {

                int py = indexedlist[i].gety();
                int px = indexedlist[i].getx();

                if(py < 0 || py > 127 || px < 0 || px > 127) continue;
                image(py, 127 - px) = yarp::sig::PixelBgr(255, 255, 255);

            }
            //drawEvents(image, stw, avgtw);
            drawEvents(image, stw, pmax.gettw());
            //drawEvents(image, stw, eventdriven::vtsHelper::maxStamp());

            drawcircle(image, avgx, avgy, avgr+0.5, 2);

            debugOut.write();
        }

        if(scopeOut.getOutputCount()) {
            yarp::os::Bottle &scopedata = scopeOut.prepare();
            scopedata.clear();
//            scopedata.addDouble(stw.front()->getStamp() * 0.001);
//            scopedata.addDouble((t - previouseventstamp) * 0.001 / 7.8125);
//            scopedata.addDouble(maxlikelihood);
//            scopedata.addDouble(avgtw / 10000.0);
//            scopedata.addDouble(pmax.gettw() / 10000.0);

            scopedata.addDouble(Tget);
            scopedata.addDouble(Tresample);
            scopedata.addDouble(Tpredict);
            scopedata.addDouble(Tobs);
            //scopedata.addDouble(1000.0 / (Tget + Tresample + Tpredict + Tobs));
            //scopedata.addDouble(eventhandler.geteventrate());
            //scopedata.addDouble(stw.size());

            scopeOut.write();
        }
    }

    std::cout << "Thread Stopped" << std::endl;
}
void particleProcessor::threadRelease()
{
    scopeOut.close();
    debugOut.close();
    eventhandler.close();
    yarp::os::Thread::threadRelease();
    std::cout << "Thread Released Successfully" <<std::endl;

}
/*////////////////////////////////////////////////////////////////////////////*/
//particleobserver (threaded observer)
/*////////////////////////////////////////////////////////////////////////////*/

vPartObsThread::vPartObsThread(int pStart, int pEnd)
{
    this->pStart = pStart;
    this->pEnd = pEnd;
}

void vPartObsThread::setDataSources(std::vector<vParticle> *particles,
                    std::vector<int> *deltats, eventdriven::vQueue *stw)
{
    this->particles = particles;
    this->deltats = deltats;
    this->stw = stw;
}

void vPartObsThread::run()
{

    for(int i = pStart; i < pEnd; i++) {
        (*particles)[i].initLikelihood();
    }

    for(int i = pStart; i < pEnd; i++) {
        for(unsigned int j = 0; j < (*stw).size(); j++) {
            if((*deltats)[j] < (*particles)[i].gettw()) {
                eventdriven::AddressEvent *v = (*stw)[j]->getUnsafe<eventdriven::AddressEvent>();
                (*particles)[i].incrementalLikelihood(v->getX(), v->getY(), (*deltats)[j]);
            } else {
                break;
            }
        }
    }

    normval = 0.0;
    for(int i = pStart; i < pEnd; i++) {
        (*particles)[i].concludeLikelihood();
        normval += (*particles)[i].getw();
    }
}

/*////////////////////////////////////////////////////////////////////////////*/
//particle reader (callback)
/*////////////////////////////////////////////////////////////////////////////*/
vParticleReader::vParticleReader(unsigned int width, unsigned int height)
{

    res.width = width; res.height = height;
    surfaceLeft = eventdriven::temporalSurface(width, height);
    strict = false;
    pmax.initWeight(0.0);
    srand(yarp::os::Time::now());

    avgx = 64;
    avgy = 64;
    avgr = 20;
    nparticles = 500;
    pwsum = 1.0;
    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);
    //rate = 100 * 7.8125;
    rate = 1000;
}

/******************************************************************************/
bool vParticleReader::open(const std::string &name, bool strictness)
{
    if(strictness) {
        this->strict = true;
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();
    if(!yarp::os::BufferedPort<eventdriven::vBottle>::open("/" + name + "/vBottle:i"))
        return false;
    if(!scopeOut.open("/" + name + "/scope:o"))
        return false;
    if(!debugOut.open("/" + name + "/debug:o"))
        return false;

    return true;
}

/******************************************************************************/
void vParticleReader::close()
{
    //close ports
    scopeOut.close();
    debugOut.close();
    yarp::os::BufferedPort<eventdriven::vBottle>::close();

}

/******************************************************************************/
void vParticleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    scopeOut.interrupt();
    debugOut.interrupt();
    yarp::os::BufferedPort<eventdriven::vBottle>::interrupt();
}



eventdriven::vQueue temporalSelection(eventdriven::vQueue &q, int ctime, int dtime)
{
    eventdriven::vQueue subq;

    if(ctime - dtime < 0) {
        for(unsigned int i = 0; i < q.size(); i++) {
            if(q[i]->getStamp() > ctime - dtime + eventdriven::vtsHelper::maxStamp() || q[i]->getStamp() < ctime)
                subq.push_back(q[i]);
        }
    } else {
        for(unsigned int i = 0; i < q.size(); i++) {
            if(q[i]->getStamp() < ctime && q[i]->getStamp() > (ctime - dtime))
                subq.push_back(q[i]);
        }
    }

    //std::cout << q.size() << " " << subq.size() << std::endl;

    return subq;

}



/******************************************************************************/
void vParticleReader::onRead(eventdriven::vBottle &inputBottle)
{

    yarp::os::Stamp st;
    getEnvelope(st);
    if(st.getCount() != pstamp.getCount() +1) {
        std::cout << "Lost Bottle" << std::endl;
    }
    pstamp = st;

    //create event queue
    eventdriven::vQueue q = inputBottle.get<eventdriven::AddressEvent>();
    //q.sort(true);

    eventdriven::vQueue stw;
    unsigned long t = 0;
    int resampled = 0;
    int processed = 0;

    for(eventdriven::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        if((*qi)->getChannel()) continue;

        surfaceLeft.addEvent(**qi);

        t = unwrap((*qi)->getStamp());

        //initialise if needed
        if(sortedlist.empty()) {
            //initialise the particles
            vParticle p;
            p.setRate(rate);

            for(int i = 0; i < nparticles; i++) {
                p.setid(i);
                p.resample(1.0/nparticles, t);
                p.initWeight(1.0/nparticles);
                sortedlist.push(p);
                indexedlist.push_back(p);
            }
        }

        //update at a fixed rate
        if(rate && indexedlist[0].needsUpdating(t)) {


            //resampling
            //if(pwsumsq * nparticles > 2.0) {

            if(true) {
                resampled = nparticles;
                std::vector<vParticle> indexedSnap = indexedlist;
                for(int i = 0; i < nparticles; i++) {
                    double rn = 1.05 * pwsum * (double)rand() / RAND_MAX;
                    if(rn > pwsum)
                        indexedlist[i].resample(1.0/nparticles, t);
                    else {
                        double accum = 0.0; int j = 0;
                        for(j = 0; j < nparticles; j++) {
                            accum += indexedSnap[j].getw();
                            if(accum > rn) break;
                        }
                        indexedlist[i].resample(indexedSnap[j], 1.0/nparticles, t);
                    }
                }
            }

            //prediction
            unsigned int maxtw = 0;
            for(int i = 0; i < nparticles; i++) {
                if(indexedlist[i].predict(t)) {
                    indexedlist[i].resample(1.0/nparticles, t);
                }
                if(indexedlist[i].gettw() > maxtw)
                    maxtw = indexedlist[i].gettw();
            }

            //likelihood observation
            for(int i = 0; i < nparticles; i++) {
                indexedlist[i].initLikelihood();
            }

            stw = surfaceLeft.getSurf_Tlim(maxtw);
            unsigned int ctime = (*qi)->getStamp();
            for(unsigned int i = 0; i < stw.size(); i++) {
                //calc dt
                double dt = ctime - stw[i]->getStamp();
                if(dt < 0)
                    dt += eventdriven::vtsHelper::maxStamp();
                eventdriven::AddressEvent *v = stw[i]->getUnsafe<eventdriven::AddressEvent>();
                int x = v->getX();
                int y = v->getY();
                for(int i = 0; i < nparticles; i++) {
                    if(dt < indexedlist[i].gettw())
                        indexedlist[i].incrementalLikelihood(x, y, dt);
                }

            }

            double normval = 0.0;
            for(int i = 0; i < nparticles; i++) {
                indexedlist[i].concludeLikelihood();
                normval += indexedlist[i].getw();
            }

            //normalisation
            pwsum = 0;
            pwsumsq = 0;
            avgx = 0;
            avgy = 0;
            avgr = 0;
            avgtw = 0;

//            double avgvx = 0;
//            double avgvy = 0;
//            double avgvr = 0;
            pmax = indexedlist[0];
            for(int i = 0; i < nparticles; i ++) {
                indexedlist[i].updateWeightSync(normval);
                if(indexedlist[i].getw() > pmax.getw()) {
                    pmax = indexedlist[i];
                }

                pwsum += indexedlist[i].getw();
                pwsumsq += pow(indexedlist[i].getw(), 2.0);
                avgx += indexedlist[i].getx() * indexedlist[i].getw();
                avgy += indexedlist[i].gety() * indexedlist[i].getw();
                avgr += indexedlist[i].getr() * indexedlist[i].getw();
                avgtw += indexedlist[i].gettw() * indexedlist[i].getw();
//                avgvx += indexedlist[i].getvx() * indexedlist[i].getw();
//                avgvy += indexedlist[i].getvy() * indexedlist[i].getw();
//                avgvr += indexedlist[i].getvr() * indexedlist[i].getw();
            }

            //pmax.initState(avgx, avgy, avgr, avgvx, avgvy, avgvr);

            //std::cout << pmax.getTemporalWindow() << std::endl;
            //surfaceLeft.setTemporalSize(maxtw);
            //indexedlist[0].setRate(avgtw / 8.0);
            indexedlist[0].initTiming(t);

        }




        vParticle p = sortedlist.top();
        while(!rate && p.needsUpdating(t)) {
            //std::cout << "Asynchronous Update" << std::endl;
            processed++;

            double oldweight = p.getw();

//            if((double)rand() / RAND_MAX < 0.1) {
//                p.resample(1.0/nparticles, t);
//            } else

            //if((1.0 / pwsumsq) < (0.5*nparticles)) {
            if(1.0 / (pwsumsq * nparticles) < (double)rand() / RAND_MAX) {
            //if(1.0 / (pwsumsq * nparticles) < p.getw()) {
            //if(false) {
                resampled++;
                double rn = (1.0*pwsum) * (double)rand() / RAND_MAX;
                if(rn > pwsum)
                    p.resample(1.0/nparticles, t);
                else {
                    double accum = 0.0; int i = 0;
                    for(i = 0; i < nparticles; i++) {
                        accum += indexedlist[i].getw();
                        if(accum > rn) break;
                    }
                    p.resample(indexedlist[i], 1.0/nparticles, t);
                    //p.resample(pmax, 1.0/nparticles, t);
                }
//                if((double)rand() / RAND_MAX < 0.1) {
//                    p.resample(1.0/nparticles, t);
//                } else {
//                    p.resample(pmax, 1.0/nparticles, t);
//                }
//                resampled = true;
            }

            if(p.predict(t)) {
                p.resample(1.0/nparticles, t);
            }


            //get the stw
            int prad = (int)(p.getr()*1.0 + 4 +0.5);
            stw = surfaceLeft.getSurf_Tlim(p.getTemporalWindow(), p.getx(), p.gety(), prad);
            //stw = temporalSelectionq(sw, (*qi)->getStamp(), p.getTemporalWindow());

            //calculate the likelihood and update weight
            //p.updateWeight(p.calcLikelihood(stw, nparticles, prad*2 + 1), pwsum);
            p.updateWeight2(p.calcLikelihood(stw, nparticles), pwsumsq);


            //update our best particle
            if(p.getw() > pmax.getw() || p.getid() == pmax.getid()) {
                pmax = p;
            }

            //update our estimate of particle weights
            pwsum += p.getw() - oldweight;
            pwsumsq += pow(p.getw(), 2.0) - pow(oldweight, 2.0);
            //std::cout << pwsum << std::endl;
            //if(pwsum > 1.0)
            //    std::cout << "PWSUM error" << std::endl;

            //remove it and push it back in
            sortedlist.pop();
            sortedlist.push(p);

            //visualisations out
            indexedlist[p.getid()] = p;

            p = sortedlist.top();
        }

    }
//    if(q.size() > 1) {
//        std::cout << (double)processed / q.size() << " particles processed / event\t|| ";
//        std::cout << (double)processed / ((q.back()->getStamp() - q.front()->getStamp())*(1e-3)*7.8125) << " particles processed / ms" << std::endl;
//    }

    yarp::os::Bottle &sob = scopeOut.prepare();
    sob.clear();
    //sob.addDouble(pwsum); sob.addDouble(pmax.getw()); sob.addDouble(1.0 - (1.0 / (pwsumsq * nparticles)));
    sob.addDouble(t);
    if(q.size())
        sob.addDouble(q.back()->getStamp());
    scopeOut.write();

    //std:: cout << sqrt(pow(pmax.getax(), 2.0) + pow(pmax.getay(), 2.0) + pow(pmax.getar(), 2.0)) << std::endl;

    //std::cout << pwsum << std::endl;
    //static int i = 0;
    //if(i++ % 50 == 0) {
    if(resampled) {
        //std::cout << pmax.getTemporalWindow() << "clock ticks" << std::endl;
        if(resampled > 255) resampled = 255;
        yarp::sig::PixelBgr pcol = yarp::sig::PixelBgr(255-resampled, 255-resampled, 255);

        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
        image.resize(res.width, res.height);
        image.zero();
        //drawcircle(image, avgx, avgy, avgr, 2);
        drawcircle(image, pmax.getx(), pmax.gety(), pmax.getr()+0.5, 2);
        //stw = surfaceLeft.getSurfLim(pmax.getTemporalWindow(), pmax.getx(), pmax.gety(), pmax.getr()+2);
        //stw = surfaceLeft.getSurf_Tlim(pmax.getTemporalWindow());
        stw = surfaceLeft.getSurf_Tlim(avgtw);
        //stw = surfaceLeft.getSurf_Clim(pmax.getr()*2.0*M_PI);
        //stw = surfaceLeft.getSurf_Clim(pmax.getr()*2.0*M_PI, pmax.getx(), pmax.gety(), pmax.getr()+2.5);
        //stw = surfaceLeft.getSurf_Tlim(t - stw.back()->getStamp());
        //stw = temporalSelection(sw, q.back()->getStamp(), pmax.getTemporalWindow());


        //drawcircle(image, pmax.getx()+pmax.getvx()*1e6, pmax.gety()+pmax.getvy()*1e6, 2, 0);

        for(unsigned int i = 0; i < indexedlist.size(); i++) {

            int py = indexedlist[i].gety();
            int px = indexedlist[i].getx();

            if(py < 0 || py > 127 || px < 0 || px > 127) continue;
            pcol = yarp::sig::PixelBgr(255*indexedlist[i].getw()/pmax.getw(), 255*indexedlist[i].getw()/pmax.getw(), 255);
            image(py, 127 - px) = pcol;
        //drawcircle(image, indexedlist[i].getx(), indexedlist[i].gety(), indexedlist[i].getr(), indexedlist[i].getid());
        }
        drawEvents(image, stw);
        //stw = surfaceLeft.getSurf_Clim(pmax.getr()*2.0*M_PI, pmax.getx(), pmax.gety(), pmax.getr() + 0.5);
        //drawEvents(image, stw);
        //drawcircle(image, pmax.getx(), pmax.gety(), pmax.getr(), pmax.getid());
        //drawcircle(image, avgx, avgy, avgr, 0);
        //std::cout << 1.0 / pwsumsq << " " << nparticles << std::endl;
        debugOut.write();

    }



}
