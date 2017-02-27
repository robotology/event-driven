#include "vRealTime.h"

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
//surfacehandler
/*////////////////////////////////////////////////////////////////////////////*/
vSurfaceHandler::vSurfaceHandler(unsigned int width, unsigned int height)
{

    res.width = width; res.height = height;
    surfaceLeft = ev::temporalSurface(width, height);
    surfaceRight = ev::temporalSurface(width, height);
    roiwinLeft.getWindow(0, width , 0, height, 50000);
    roiwinRight.getWindow(0, width , 0, height, 50000);
    strict = false;
    pstamp = yarp::os::Stamp();
    ptime = yarp::os::Time::now();
    condTime = 0;
    tw = 0;
    eventrate = 0;
    bottletime = yarp::os::Time::now();
    eventsQueried = false;
    waitsignal.wait(); //lock the resource to start with
    vcount = 0;

}

void vSurfaceHandler::resize(unsigned int width, unsigned int height)
{
    res.width = width; res.height = height;
    surfaceLeft = ev::temporalSurface(width, height);
    surfaceRight = ev::temporalSurface(width, height);
}

bool vSurfaceHandler::open(const std::string &name, bool strictness)
{
    if(strictness) {
        this->strict = true;
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();
    if(!yarp::os::BufferedPort<ev::vBottle>::open(name + "/vBottle:i"))
        return false;

    return true;
}

void vSurfaceHandler::close()
{
    yarp::os::BufferedPort<ev::vBottle>::close();
}

void vSurfaceHandler::interrupt()
{
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

void vSurfaceHandler::onRead(ev::vBottle &inputBottle)
{
    yarp::os::Stamp st;
    getEnvelope(st);
    if(st.getCount() != pstamp.getCount() +1 && pstamp.isValid()) {
        std::cout << "Lost Bottle" << std::endl;
    }
    pstamp = st;

    //create event queue
    //static double accum = 0;
    //double temp1 = yarp::os::Time::now();
    ev::vQueue q = inputBottle.get<AddressEvent>();
    //accum += (yarp::os::Time::now() - temp1);
    //temp1 = yarp::os::Time::now() - temp1;
    //std::cout << accum << std::endl;
    //q.sort(true);

   // double temp2 = yarp::os::Time::now();
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        mutexsignal.lock();

        vcount++;


        if((*qi)->getChannel() == 0)
            surfaceLeft.fastAddEvent(*qi);
        //roiwinLeft.addEvent(std::static_pointer_cast<AddressEvent>(*qi));
        else if((*qi)->getChannel() == 1)
            surfaceRight.fastAddEvent(*qi);
        //roiwinRight.addEvent(std::static_pointer_cast<AddressEvent>(*qi));
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

    //accum += (yarp::os::Time::now() - temp2);



   // std::cout << temp1*1000 << " " << accum*1000 << std::endl;
    //accum = 0;



}

double vSurfaceHandler::queryROI(ev::vQueue &fillq, unsigned int temporalwindow, int x, int y, int r)
{

    //std::cout << vcount << std::endl;
    if(!vcount) return 0;

    tw = temporalwindow;

    mutexsignal.lock();

    //surfaceLeft.getSurfSorted(fillq);
    //fillq = roiwinLeft.getWindow(x - r, x + r, y - r, y + r, temporalwindow);
    //fillq = surfaceRight.getSurf_Tlim(temporalwindow, x, y, r);
    double t1 = yarp::os::Time::now();
    fillq = surfaceLeft.getSurf_Tlim(temporalwindow, x, y, r);
    double t2 = yarp::os::Time::now();
    //fillq.sort(true);


    vcount = 0;

    mutexsignal.unlock();

    return t2 - t1;


}


void vSurfaceHandler::queryEvents(ev::vQueue &fillq, unsigned int temporalwindow)
{
    tw = temporalwindow;

    mutexsignal.lock();
    //surfaceLeft.getSurfSorted(fillq);
    //fillq = surfaceRight.getSurf_Tlim(temporalwindow);
    fillq = surfaceRight.getSurf_Tlim(temporalwindow);
    mutexsignal.unlock();

}

ev::vQueue vSurfaceHandler::queryEvents(unsigned long int conditionTime, unsigned int temporalWindow)
{



    condTime = conditionTime;
    tw = temporalWindow;

//    if(false) { //tnow < condTime
//        eventsQueried = true;
//        waitsignal.wait();
//    }

    mutexsignal.lock();
    //ev::vQueue temp = surfaceLeft.getSurf_Tlim(tw);
    ev::vQueue temp;
    surfaceRight.getSurfSorted(temp);
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
particleProcessor::particleProcessor(unsigned int height, unsigned int width, std::string name, bool strict)
{
    res.height = height;
    res.width  = width;
    this->name = name;
    this->strict = strict;
    ptime2 = yarp::os::Time::now();

    nparticles = 50;
    nThreads = 2;
    rate = 0;
    nRandomise = 1.0 + 0.02;
    adaptive = false;

    obsInlier = 1.5;
    obsOutlier = 3.0;
    obsThresh = 20.0;

    avgx = 10;
    avgy = 10;
    avgr = 10;
    avgtw = 100;
    maxtw = 10000;
    pwsumsq = 0;
    maxlikelihood = 1;
    pVariance = 0.5;

    eventhandler.resize(width, height);
    eventhandler2.configure(height, width);

}

bool particleProcessor::threadInit()
{
    std::cout << "Initialising thread" << std::endl;

    for(int i = 0; i < nThreads; i++) {
        int pStart = i * (nparticles / nThreads);
        int pEnd = (i+1) * (nparticles / nThreads);
        if(i == nThreads - 1)
            pEnd = nparticles;

        std::cout << pStart << "->" << pEnd-1 << std::endl;
        computeThreads.push_back(new vPartObsThread(pStart, pEnd));
    }

    if(!debugOut.open(name + "/debug:o")) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    if(!scopeOut.open(name + "/scope:o")) {
        std::cout << "could not open scope port" << std::endl;
        return false;
    }
    if(!vBottleOut.open(name + "/vBottle:o")) {
        std::cout << "coult not open vBottleOut port" << std::endl;
        return false;
    }

    if(!eventhandler2.open(name + "/vBottle:i")) {
        std::cout << "Could not open eventhandler2 ports" << std::endl;
        return false;
    }


//    if(!eventhandler.open(name, strict)) {
//        std::cout << "Could not open eventhandler ports" << std::endl;
//        return false;
//    }

    //std::cout << "Port open - querying initialisation events" << std::endl;
    //ev::vQueue stw;
    //while(!stw.size()) {
    //       yarp::os::Time::delay(0.01);
    //       stw = eventhandler.queryEvents(0, 1);
    //}

    //unsigned int tnow = unwrap(stw.back()->getStamp());


    //initialise the particles
    vParticle p;
    p.setRate(rate);
    p.initWeight(1.0/nparticles);
    p.setInlierParameter(obsInlier);
    p.setOutlierParameter(obsOutlier);
    p.setMinLikelihood(obsThresh);
    p.setVariance(pVariance);

    indexedlist.clear();
    for(int i = 0; i < nparticles; i++) {
        p.setid(i);
        p.resample(1.0/nparticles, 0, res.width, res.height, 30);
        maxtw = std::max(maxtw, p.gettw());
        indexedlist.push_back(p);
    }

    std::cout << "Thread initialised" << std::endl;
    return true;

}

void particleProcessor::run()
{
    std::cout << "Thread starting" << std::endl;

    ev::vQueue stw, stw2;
    unsigned long int t;
    //double maxlikelihood;
    while(!stw2.size()) {
        yarp::os::Time::delay(0.1);
        eventhandler2.queryROI(stw2, 1, 100000, res.width/2, res.height/2, res.width/2);
    }
    //t = unwrap(stw2.front()->getStamp());

    while(!isStopping()) {
        ptime = yarp::os::Time::now();

            //stw.swap(stw2);
        //stw.swap(stw2);
        stw = stw2;
        //stw2.clear();

        //std::cout << "GETTING EVENTS" <<std::endl;

        //eventhandler.queryEvents(stw, maxtw);
        //eventhandler2.queryROI(stw, 0, maxtw, avgx, avgy, avgr * 1.5);
        //std::cout << stw.size() << std::endl;
        //eventdriven::vQueue stw = eventhandler.queryEvents(unwrap.currentTime() + rate, eventdriven::vtsHelper::maxStamp() * 0.5);
        //eventdriven::vQueue stw = eventhandler.queryEventList(indexedlist);

        double Tget = (yarp::os::Time::now() - ptime)*1000.0;
        ptime = yarp::os::Time::now();

        //if(!stw.size()) continue;

        if(stw.size())
            t = unwrap(stw.front()->getStamp());

        std::vector<vParticle> indexedSnap = indexedlist;

        //resampling
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            for(int i = 0; i < nparticles; i++) {
                //if(indexedlist[i].getw() > (0.1 / nparticles)) continue;
                double rn = nRandomise * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].resample(1.0/nparticles, t, res.width, res.height, 30.0);
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
            indexedlist[i].predict(t);
            if(!inbounds(indexedlist[i]))
                indexedlist[i].resample(1.0/nparticles, t, res.width, res.height, 30.0);

            //if(indexedlist[i].predict(t))
            //    indexedlist[i].resample(1.0/nparticles, t);
            //else
            //    if(!inbounds(indexedlist[i]))
            //        std::cout << "Inbounds not computed correctly " << indexedlist[i].getx() << " " << indexedlist[i].gety() << " " << indexedlist[i].getr() << std::endl;

            //if(!inbounds(indexedlist[i]))
            if(indexedlist[i].gettw() > maxtw)
                maxtw = indexedlist[i].gettw();
        }

        double Tpredict = (yarp::os::Time::now() - ptime)*1000.0;
        ptime = yarp::os::Time::now();

        //likelihood observation
        std::vector<int> deltats; deltats.resize(stw.size());
        double stampnow = stw.front()->stamp;
        for(unsigned int i = 0; i < stw.size(); i++) {
            double dt = stampnow - stw[i]->stamp;
            if(dt < 0)
                dt += ev::vtsHelper::max_stamp;
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
//                    event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(stw[i]);
//                    indexedlist[j].incrementalLikelihood(v->getX(), v->getY(), deltats[i]);
//                } else {
//                    break;
//                }
//            }
//        }

//        //maxlikelihood = 0;
//        double normval = 0.0;
//        for(int i = 0; i < nparticles; i++) {
//            indexedlist[i].concludeLikelihood();
//            normval += indexedlist[i].getw();
//            //maxlikelihood = std::max(maxlikelihood, indexedlist[i].getl());
//        }

        //END SINGLE-THREAD

        //START MULTI-THREAD
        for(int k = 0; k < nThreads; k++) {
            computeThreads[k]->setDataSources(&indexedlist, &deltats, &stw);
            computeThreads[k]->start();
        }

        //std::cout << "getting new event window" << std::endl;
        eventhandler2.queryROI(stw2, 1, maxtw, avgx, avgy, avgr * 1.5);

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



        //std::cout << avgx << " " << avgy << " " << avgr << " " << avgtw << std::endl;
        //std::cout << "CALCULATED AVERAGE" <<std::endl;

//        particleVariance = 0;
//        for(int i = 0; i < nparticles; i ++) {
//            //particleVariance = indexedlist[i].getw() *
//            particleVariance =
//                    (pow(avgx - indexedlist[i].getx(), 2.0) + pow(avgy - indexedlist[i].gety(), 2.0));
//        }
//        particleVariance /= nparticles;


        if(vBottleOut.getOutputCount()) {
            ev::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
            ev::event<ev::ClusterEventGauss> ceg = ev::event<ev::ClusterEventGauss>(new ev::ClusterEventGauss());
            ceg->setStamp(stw.front()->getStamp());
            ceg->setChannel(1);
            ceg->setID(0);
            ceg->setNumAE(0);
            ceg->setPolarity(0);
            ceg->setXCog(avgx);
            ceg->setYCog(avgy);
            ceg->setXSigma2(avgr);
            ceg->setYSigma2(1);
            ceg->setXVel(0);
            ceg->setYVel(0);
            ceg->setXYSigma(0);

            eventsout.addEvent(ceg);
            vBottleOut.write();
        }

        //static int i = 0;
        double Timage = yarp::os::Time::now();
        if(debugOut.getOutputCount()) {

            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
            image.resize(res.width, res.height);
            image.zero();

            for(unsigned int i = 0; i < indexedlist.size(); i++) {

                int py = indexedlist[i].gety();
                int px = indexedlist[i].getx();

                if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
                image(px, py) = yarp::sig::PixelBgr(255, 255, 255);

            }
            //drawEvents(image, stw, avgtw);
            drawEvents(image, stw, pmax.gettw());
            //drawEvents(image, stw, eventdriven::vtsHelper::maxStamp());

            drawcircle(image, avgx, avgy, avgr+0.5, 2);



            debugOut.write();
        }
        Timage = (yarp::os::Time::now() - Timage)*1000;

        if(scopeOut.getOutputCount()) {
            yarp::os::Bottle &scopedata = scopeOut.prepare();
            scopedata.clear();
            double temptime = yarp::os::Time::now();
            scopedata.addDouble(1.0 / (temptime - ptime2));
            //scopedata.addDouble(avgr);
            ptime2 = temptime;
//            scopedata.addDouble(stw.front()->getStamp() * 0.001);
//            scopedata.addDouble((t - previouseventstamp) * 0.001 / 7.8125);
//            scopedata.addDouble(maxlikelihood);
//            scopedata.addDouble(avgtw / 10000.0);
//            scopedata.addDouble(pmax.gettw() / 10000.0);

//            scopedata.addDouble(Tget);
//            scopedata.addDouble(Tresample);
//            scopedata.addDouble(Timage);
//            scopedata.addDouble(Tobs);
            //scopedata.addDouble(1000.0 / (Tget + Tresample + Tpredict + Tobs));
            //scopedata.addDouble(eventhandler.geteventrate());
            //scopedata.addDouble(stw.size());

            scopeOut.write();
        }

    }

    std::cout << "Thread Stopped" << std::endl;
}

bool particleProcessor::inbounds(vParticle &p)
{
    int r = p.getr();
    int minr = res.width/18;
    int maxr = res.width/5;
    if(r < minr) {
        p.setr(minr);
        r = minr;
    }
    if(r > maxr) {
        p.setr(maxr);
        r = maxr;
    }
    if(p.getx() < -r || p.getx() > res.width + r)
        return false;
    if(p.gety() < -r || p.gety() > res.height + r)
        return false;

    return true;
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
                    std::vector<int> *deltats, ev::vQueue *stw)
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
                //ev::AddressEvent *v = (*stw)[j]->getUnsafe<ev::AddressEvent>();
                event<AddressEvent> v = std::static_pointer_cast<AddressEvent>((*stw)[j]);
                //event<AddressEvent> v = getas<AddressEvent>((*stw)[j]);
                (*particles)[i].incrementalLikelihood(v->x, v->y, (*deltats)[j]);
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
