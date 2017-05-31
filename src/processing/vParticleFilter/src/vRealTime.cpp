#include "vRealTime.h"

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
//particleprocessor (real-time)
/*////////////////////////////////////////////////////////////////////////////*/
particleProcessor::particleProcessor(std::string name, unsigned int height, unsigned int width, hSurfThread* eventhandler, collectorPort *eventsender)
{
    res.height = height;
    res.width  = width;
    this->eventhandler = eventhandler;
    this->eventsender = eventsender;
    ptime2 = yarp::os::Time::now();
    pytime = 0;
    this->name = name;

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

    avgx = 64;
    avgy = 64;
    avgr = 12;
    avgtw = 100;
    maxtw = 10000;
    pwsumsq = 0;
    maxlikelihood = 1;
    pVariance = 0.5;
    rbound_max = 50;
    rbound_min = 10;

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

    rbound_min = res.width/18;
    rbound_max = res.width/6;

    pcb.configure(res.height, res.width, rbound_max, 128);

    if(camera == 1) {
        if(!scopeOut.open(name + "/scope:o")) {
            std::cout << "could not open scope port" << std::endl;
            return false;
        }
    }

    //initialise the particles
    vParticle p;

    indexedlist.clear();
    for(int i = 0; i < nparticles; i++) {
        p.initialiseParameters(i, obsThresh, obsOutlier, obsInlier, pVariance, 128);
        p.attachPCB(&pcb);

        if(seedr)
            p.initialiseState(seedx, seedy, seedr, 50000);
        else
            p.randomise(res.width, res.height, 30, 50000);

        p.resetWeight(1.0/nparticles);

        maxtw = std::max(maxtw, p.gettw());
        indexedlist.push_back(p);
    }

    yInfo() << "Thread and particles initialised";
    return true;

}

void particleProcessor::run()
{
    yInfo() << "VPF Thread starting for camera: " << camera;

    ptime2 = yarp::os::Time::now();
    ev::vQueue stw, stw2;
    unsigned long int pt = 0;
    unsigned long int t = 0;
    //double maxlikelihood;
    while(!stw2.size() && !isStopping()) {
        yarp::os::Time::delay(0.1);
        if(useroi)
            stw2 = eventhandler->queryROI(camera, 100000, res.width/2, res.height/2, res.width/2);
            //yarpstamp = eventhandler2.queryROI(stw2, camera, 100000, res.width/2, res.height/2, res.width/2);
        else
            stw2 = eventhandler->queryWindow(camera, 100000);
            //yarpstamp = eventhandler2.queryWindow(stw2, camera, 100000);
    }
    yarp::os::Stamp yarpstamp = eventhandler->queryYstamp();
    int currentstamp = eventhandler->queryVstamp(camera);

    while(!isStopping()) {

        stw = stw2;

        pt = t;
        t = unwrap(currentstamp);

        std::vector<vParticle> indexedSnap = indexedlist;

        //resampling
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            for(int i = 0; i < nparticles; i++) {
                double rn = nRandomise * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].randomise(res.width, res.height, 30.0, avgtw);
                else {
                    double accum = 0.0; int j = 0;
                    for(j = 0; j < nparticles; j++) {
                        accum += indexedSnap[j].getw();
                        if(accum > rn) break;
                    }
                    indexedlist[i] = indexedSnap[j];
                }
            }
        }

        //prediction
        maxtw = 0; //also calculate maxtw for next processing step
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].predict(t);
            if(!inbounds(indexedlist[i]))
                indexedlist[i].randomise(res.width, res.height, 30.0, avgtw);

            if(indexedlist[i].gettw() > maxtw)
                maxtw = indexedlist[i].gettw();
        }


        //likelihood observation
        std::vector<int> deltats; deltats.resize(stw.size());
        for(unsigned int i = 0; i < stw.size(); i++) {
            double dt = currentstamp - stw[i]->stamp;
            if(dt < 0)
                dt += ev::vtsHelper::max_stamp;
            deltats[i] = dt;
        }


        //START MULTI-THREAD
        //yarp::sig::ImageOf <yarp::sig::PixelBgr> likedebug;
        //likedebug.resize(nparticles * 4, stw.size());
        //likedebug.zero();
        for(int k = 0; k < nThreads; k++) {
            //computeThreads[k]->setDataSources(&indexedlist, &deltats, &stw, &likedebug);
            computeThreads[k]->setDataSources(&indexedlist, &deltats, &stw, 0);
            computeThreads[k]->start();
        }


        if(useroi)
            stw2 = eventhandler->queryROI(camera, maxtw, avgx, avgy, avgr * 1.5);
        else
            stw2 = eventhandler->queryWindow(camera, maxtw);

        yarpstamp = eventhandler->queryYstamp();
        currentstamp = eventhandler->queryVstamp(camera);

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

        auto ceg = make_event<GaussianAE>();
        ceg->stamp = currentstamp;
        ceg->setChannel(camera);
        ceg->x = avgx;
        ceg->y = avgy;
        ceg->sigx = avgr;
        ceg->sigy = avgr;
        ceg->sigxy = 0;
        ceg->polarity = 1;

        eventsender->pushevent(ceg, yarpstamp);

//        if(vBottleOut.getOutputCount()) {
//            ev::vBottle &eventsout = vBottleOut.prepare();
//            eventsout.clear();

//            eventsout.addEvent(ceg);
//            vBottleOut.setEnvelope(yarpstamp);
//            vBottleOut.write();
//        }

//        double Timage = yarp::os::Time::now();
//        double ydt = yarpstamp.getTime() - pytime;
//        if(debugOut.getOutputCount() && (ydt > 0.03 || ydt < 0)) {

//            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
//            image.resize(res.width, res.height);
//            image.zero();

//            for(unsigned int i = 0; i < indexedlist.size(); i++) {

//                int py = indexedlist[i].gety();
//                int px = indexedlist[i].getx();

//                if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
//                //image(res.width-1 - px, res.height - 1 - py) = yarp::sig::PixelBgr(255, 255, 255);

//            }
//            drawEvents(image, stw, currentstamp, pmax.gettw(), false);
//            //drawEvents(image, stw, pmax.gettw());
//            //drawEvents(image, stw, eventdriven::vtsHelper::maxStamp());

//            //drawcircle(image, res.width-1 - avgx, res.height-1 - avgy, avgr+0.5, 1);

//            pytime = yarpstamp.getTime();

//            //image = likedebug;
//            //drawDistribution(image, indexedlist);

//            debugOut.setEnvelope(yarpstamp);
//            debugOut.write();
//        }
//        Timage = (yarp::os::Time::now() - Timage)*1000;

        if(scopeOut.getOutputCount()) {
            yarp::os::Bottle &scopedata = scopeOut.prepare();
            scopedata.clear();

            //scopedata.addDouble(eventhandler->queryDelay(camera));

            double temptime = yarp::os::Time::now();
            scopedata.addDouble(1.0 / (temptime - ptime2));
            ptime2 = temptime;

            //scopedata.addDouble((t - pt) * vtsHelper::tsscaler);

//            scopedata.addDouble(dtnezero / (double)(dtnezero + dtezero));
//            if(dtnezero + dtezero > 1000) {
//                dtnezero = 0;
//                dtezero = 0;
//            }

            //scopedata.addDouble(stw.size());
            //scopedata.addDouble(pmax.dtvar);
            //scopedata.addDouble(pmax.gettw() * vtsHelper::tsscaler);


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
        p.resetRadius(rbound_min);
        r = rbound_min;
    }
    if(r > rbound_max) {
        p.resetRadius(rbound_max);
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
                    std::vector<int> *deltats, ev::vQueue *stw, yarp::sig::ImageOf < yarp::sig::PixelBgr> *debugIm)
{
    this->particles = particles;
    this->deltats = deltats;
    this->stw = stw;
    this->debugIm = debugIm;
}

void vPartObsThread::run()
{

    for(int i = pStart; i < pEnd; i++) {
        (*particles)[i].initLikelihood();
    }

    for(int i = pStart; i < pEnd; i++) {
        for(unsigned int j = 0; j < (*stw).size(); j++) {
            if((*deltats)[j] < (*particles)[i].gettw()) {
                auto v = is_event<AE>((*stw)[j]);
                int l = 2 * (*particles)[i].incrementalLikelihood(v->x, v->y, (*deltats)[j]);
                if(debugIm) {
                    l += 128;
                    if(l > 255) l = 255;
                    if(l < 0) l = 0;
                    (*debugIm)(i*4 + 0, j) = yarp::sig::PixelBgr(l, l, l);
                    (*debugIm)(i*4 + 1, j) = yarp::sig::PixelBgr(l, l, l);
                    (*debugIm)(i*4 + 2, j) = yarp::sig::PixelBgr(l, l, l);
                    (*debugIm)(i*4 + 3, j) = yarp::sig::PixelBgr(l, l, l);
                }
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
