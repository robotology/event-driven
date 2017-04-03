#include "vRealTime.h"

using namespace ev;

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
    pytime = 0;

    nparticles = 50;
    nThreads = 2;
    rate = 0;
    nRandomise = 1.0 + 0.02;
    adaptive = false;

    obsInlier = 1.5;
    obsOutlier = 3.0;
    obsThresh = 20.0;

    camera = 0;
    useroi = false;
    seedx = 0;
    seedy = 0;
    seedr = 0;

    avgx = 10;
    avgy = 10;
    avgr = 10;
    avgtw = 100;
    maxtw = 10000;
    pwsumsq = 0;
    maxlikelihood = 1;
    pVariance = 0.5;
    rbound_max = 50;
    rbound_min = 10;

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

    rbound_min = res.width/16;
    rbound_max = res.width/7;

    pcb.configure(res.height, res.width, rbound_max, 128);

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
        p.setPCB(&pcb);
        p.resample(1.0/nparticles, 0, res.width, res.height, 30, avgtw);
        if(seedr)
            p.initState(seedx, seedy, seedr, 100);
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
    yarp::os::Stamp yarpstamp;
    //double maxlikelihood;
    while(!stw2.size() && !isStopping()) {
        yarp::os::Time::delay(0.1);
        if(useroi)
            yarpstamp = eventhandler2.queryROI(stw2, camera, 100000, res.width/2, res.height/2, res.width/2);
        else
            yarpstamp = eventhandler2.queryWindow(stw2, camera, 100000);
    }
    //t = unwrap(stw2.front()->getStamp());

    while(!isStopping()) {
        ptime = yarp::os::Time::now();

            //stw.swap(stw2);
        //stw.swap(stw2);
//        if(useroi)
//            yarpstamp = eventhandler2.queryROI(stw2, camera, maxtw, avgx, avgy, avgr * 1.5);
//        else
//            yarpstamp = eventhandler2.queryWindow(stw2, camera, maxtw);
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
            t = unwrap(eventhandler2.queryVTime());

        std::vector<vParticle> indexedSnap = indexedlist;

        //resampling
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            for(int i = 0; i < nparticles; i++) {
                //if(indexedlist[i].getw() > (0.1 / nparticles)) continue;
                double rn = nRandomise * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].resample(1.0/nparticles, t, res.width, res.height, 30.0, avgtw);
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
                indexedlist[i].resample(1.0/nparticles, t, res.width, res.height, 30.0, avgtw);

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
//                    indexedlist[j].incrementalLikelihood(v->x, v->y, deltats[i]);
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
        if(useroi)
            yarpstamp = eventhandler2.queryROI(stw2, camera, maxtw, avgx, avgy, avgr * 1.5);
        else
            yarpstamp = eventhandler2.queryWindow(stw2, camera, maxtw);

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

        if(vBottleOut.getOutputCount()) {
            ev::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
            //ev::event<ev::ClusterEventGauss> ceg = ev::event<ev::ClusterEventGauss>(new ev::ClusterEventGauss());
            auto ceg = make_event<GaussianAE>();
            ceg->stamp = stw.front()->stamp;
            ceg->setChannel(camera);
            ceg->x = avgx;
            ceg->y = avgy;
            ceg->sigx = avgr;
            ceg->sigy = avgr;
            ceg->sigxy = 0;
            ceg->polarity = 1;
            eventsout.addEvent(ceg);
            vBottleOut.setEnvelope(yarpstamp);
            vBottleOut.write();
        }

        double Timage = yarp::os::Time::now();
        double ydt = yarpstamp.getTime() - pytime;
        if(debugOut.getOutputCount() && (ydt > 0.03 || ydt < 0)) {

            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
            image.resize(res.width, res.height);
            image.zero();

            for(unsigned int i = 0; i < indexedlist.size(); i++) {

                int py = indexedlist[i].gety();
                int px = indexedlist[i].getx();

                if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
                image(res.width-1 - px, res.height - 1 - py) = yarp::sig::PixelBgr(255, 255, 255);

            }
            drawEvents(image, stw, avgtw, true);
            //drawEvents(image, stw, pmax.gettw());
            //drawEvents(image, stw, eventdriven::vtsHelper::maxStamp());

            drawcircle(image, res.width-1 - avgx, res.height-1 - avgy, avgr+0.5, 1);

            pytime = yarpstamp.getTime();
            debugOut.setEnvelope(yarpstamp);
            debugOut.write();
        }
        Timage = (yarp::os::Time::now() - Timage)*1000;

        if(scopeOut.getOutputCount()) {
            yarp::os::Bottle &scopedata = scopeOut.prepare();
            scopedata.clear();
            double temptime = yarp::os::Time::now();
            scopedata.addDouble(1.0 / (temptime - ptime2));
            ptime2 = temptime;

//            scopedata.addDouble(avgr);
//            scopedata.addDouble(Tget);
//            scopedata.addDouble(avgtw * 10e-6);
//            scopedata.addDouble(maxtw * 10e-6);
//            scopedata.addDouble(Tobs);


            scopeOut.write();
        }

    }

    std::cout << "Thread Stopped" << std::endl;
}

bool particleProcessor::inbounds(vParticle &p)
{
    int r = p.getr();

    if(r < rbound_min) {
        p.setr(rbound_min);
        r = rbound_min;
    }
    if(r > rbound_max) {
        p.setr(rbound_max);
        r = rbound_max;
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
    eventhandler2.stop();
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
                //event<AddressEvent> v = as_event<AddressEvent>((*stw)[j]);
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
