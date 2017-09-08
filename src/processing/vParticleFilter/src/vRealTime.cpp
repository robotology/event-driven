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
        computeThreads[i]->start();
    }

    rbound_min = res.width/17;
    rbound_max = res.width/6;

    pcb.configure(res.height, res.width, rbound_max, 64);

    if(camera == 1) {
        if(!scopeOut.open(name + "/scope:o")) {
            yError() << "Could not open scope port";
            return false;
        }
        if(!debugOut.open(name + "/debug:o")) {
            yError() << "Could not open debug image output port";
            return false;
        }
    }

    //initialise the particles
    vParticle p;

    indexedlist.clear();
    for(int i = 0; i < nparticles; i++) {
        p.initialiseParameters(i, obsThresh, obsOutlier, obsInlier, pVariance, 64);
        p.attachPCB(&pcb);

        if(seedr)
            p.initialiseState(seedx, seedy, seedr, 0.01 * vtsHelper::vtsscaler);
        else
            p.randomise(res.width, res.height, rbound_max, 0.01 * vtsHelper::vtsscaler);

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

    double Twincopy = 0;
    double Tresample = 0;
    double Tpredict = 0;
    double Tlikelihood = 0;
    double Tgetwindow = 0;


    double maxlikelihood = 0;
    double stagnantstart = 0;
    bool detection = false;
    ptime2 = yarp::os::Time::now();
    ev::vQueue stw, stw2;
    int smoothcount = 1e6;
    double val1 = 0, val2 = 0, val3 = 0, val4 = 0, val5 = 0;
    //unsigned long int pt = 0;
    unsigned long int t = 0;
    int pvstamp = 0;
    //double maxlikelihood;
    while(!stw2.size() && !isStopping()) {
        yarp::os::Time::delay(0.1);
        //if(useroi)
            //stw2 = eventhandler->queryROI(camera, 100000, res.width/2, res.height/2, res.width/2);
            //yarpstamp = eventhandler2.queryROI(stw2, camera, 100000, res.width/2, res.height/2, res.width/2);
        //else
            stw2 = eventhandler->queryWindow(camera, 0.01 * vtsHelper::vtsscaler);
            //yarpstamp = eventhandler2.queryWindow(stw2, camera, 100000);
    }
    yarp::os::Stamp yarpstamp = eventhandler->queryYstamp();
    int currentstamp = eventhandler->queryVstamp(camera);

    while(!isStopping()) {

        Twincopy = yarp::os::Time::now();
        stw = stw2;
        Twincopy = yarp::os::Time::now() - Twincopy;


        //pt = t;
        t = unwrap(currentstamp);



        //resampling
        Tresample = yarp::os::Time::now();
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            std::vector<vParticle> indexedSnap = indexedlist;
            for(int i = 0; i < nparticles; i++) {
                double rn = nRandomise * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].randomise(res.width, res.height, rbound_max, 0.001 * vtsHelper::vtsscaler);
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
        Tresample = yarp::os::Time::now() - Tresample;

        //prediction
        Tpredict = yarp::os::Time::now();
        maxtw = 0; //also calculate maxtw for next processing step
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].predict(t);
            if(!inbounds(indexedlist[i]))
                indexedlist[i].randomise(res.width, res.height, rbound_max, avgtw);

            if(indexedlist[i].gettw() > maxtw)
                maxtw = indexedlist[i].gettw();
        }
        Tpredict = yarp::os::Time::now() - Tpredict;

        //likelihood observation
        Tlikelihood = yarp::os::Time::now();
        std::vector<int> deltats; deltats.resize(stw.size());
        for(unsigned int i = 0; i < stw.size(); i++) {
            double dt = currentstamp - stw[i]->stamp;
            if(dt < 0)
                dt += ev::vtsHelper::max_stamp;
            deltats[i] = dt;
        }

        double normval = 0.0;
        if(nThreads == 1) {
            //START WITHOUT THREAD

            for(int i = 0; i < nparticles; i++) {
                indexedlist[i].initLikelihood();
            }

            int ntoproc = std::min((int)(stw).size(), 300);

            for(int i = 0; i < nparticles; i++) {
                for(unsigned int j = 0; j < ntoproc; j++) {
                    AE* v = read_as<AE>(stw[j]);
                    indexedlist[i].incrementalLikelihood(v->x, v->y, deltats[j]);
                }
            }

            for(int i = 0; i < nparticles; i++) {
                indexedlist[i].concludeLikelihood();
                normval += indexedlist[i].getw();
            }

        } else {


            //START MULTI-THREAD
            //yarp::sig::ImageOf <yarp::sig::PixelBgr> likedebug;
            //likedebug.resize(nparticles * 4, stw.size());
            //likedebug.zero();
            for(int k = 0; k < nThreads; k++) {
                //computeThreads[k]->setDataSources(&indexedlist, &deltats, &stw, &likedebug);
                computeThreads[k]->setDataSources(&indexedlist, &deltats, &stw, 0);
                //computeThreads[k]->start();
                computeThreads[k]->process();
            }

            for(int k = 0; k < nThreads; k++) {
                //computeThreads[k]->join();
                //normval += computeThreads[k]->getNormVal();
                normval += computeThreads[k]->waittilldone();
            }
        }


        //END MULTI-THREAD

        //normalisation

        pwsumsq = 0;
        vParticle pmax = indexedlist[0];
        maxlikelihood = 0;
        for(int i = 0; i < nparticles; i ++) {
            indexedlist[i].updateWeightSync(normval);
            pwsumsq += pow(indexedlist[i].getw(), 2.0);
            if(indexedlist[i].getw() > pmax.getw())
                pmax = indexedlist[i];
            maxlikelihood = std::max(maxlikelihood, indexedlist[i].getl());
        }
        Tlikelihood = yarp::os::Time::now() - Tlikelihood;


        //grab the new events in parallel as computing the likelihoods
        Tgetwindow = yarp::os::Time::now();
        if(useroi)
            stw2 = eventhandler->queryROI(camera, maxtw, avgx, avgy, avgr * 1.5);
        else
            stw2 = eventhandler->queryWindow(camera, maxtw);

        yarpstamp = eventhandler->queryYstamp();
        currentstamp = eventhandler->queryVstamp(camera);
        Tgetwindow = yarp::os::Time::now() - Tgetwindow;


        //check for stagnancy
        if(maxlikelihood < 32.0) {

            if(!stagnantstart) {
                stagnantstart = yarp::os::Time::now();
            } else {
                if(yarp::os::Time::now() - stagnantstart > 1.0) {
                    for(int i = 0; i < nparticles; i++) {
                        indexedlist[i].initialiseState(res.width/2.0,
                                                       res.height/2.0,
                                                       rbound_min + (rbound_max - rbound_min) * ((double)rand()/RAND_MAX),
                                                        0.001 * vtsHelper::vtsscaler);
                    }
                    detection = false;
                    stagnantstart = 0;
                    yInfo() << "Performing full resample";
                }
            }
        } else {
            detection = true;
            stagnantstart = 0;
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
        ceg->sigxy = obsInlier;
        ceg->polarity = detection;

        eventsender->pushevent(ceg, yarpstamp);


        double imagedt = yarp::os::Time::now() - pytime;
        if(debugOut.getOutputCount() && (imagedt > 0.03 || imagedt < 0)) {

            pytime = yarp::os::Time::now();

            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
            image.resize(res.width, res.height);
            image.zero();

            for(unsigned int i = 0; i < indexedlist.size(); i++) {

                int py = indexedlist[i].gety();
                int px = indexedlist[i].getx();

                if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
                image(res.width-1 - px, res.height - 1 - py) = yarp::sig::PixelBgr(255, 255, 255);

            }
            drawEvents(image, stw, currentstamp, avgtw, true);

            drawcircle(image, res.width-1 - avgx, res.height-1 - avgy, avgr+0.5, 1);

            debugOut.setEnvelope(yarpstamp);
            debugOut.write();
        }

        static double pscopetime = yarp::os::Time::now();
        if(scopeOut.getOutputCount()) {

            val1 = std::max(val1, Twincopy);
            val2 = std::max(val2, Tresample);
            val3 = std::max(val3, Tpredict);
            val4 = std::max(val4, Tlikelihood);
            val5 = std::max(val5, Tgetwindow);


//            double temptime = yarp::os::Time::now();
//            val1 = std::max(val1, (temptime - ptime2));
//            ptime2 = temptime;

//            val2 = std::max(val2, eventhandler->queryDelay((camera)));
//            val3 = std::max(val3, maxlikelihood);

//            double vdt = currentstamp - pvstamp;
//            pvstamp = currentstamp;
//            if(vdt < 0) vdt += ev::vtsHelper::max_stamp;
//            val4 = std::max(val4, vdt * ev::vtsHelper::tsscaler);

            double scopedt = yarp::os::Time::now() - pscopetime;
            if((scopedt > 0.05 || scopedt < 0)) {
                pscopetime += scopedt;

                yarp::os::Bottle &scopedata = scopeOut.prepare();
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

                scopeOut.write();
            }
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
    processing.lock();
    done.lock();
}

void vPartObsThread::setDataSources(std::vector<vParticle> *particles,
                    std::vector<int> *deltats, ev::vQueue *stw, yarp::sig::ImageOf < yarp::sig::PixelBgr> *debugIm)
{
    this->particles = particles;
    this->deltats = deltats;
    this->stw = stw;
    this->debugIm = debugIm;
}

void vPartObsThread::process()
{
    processing.unlock();
}

double vPartObsThread::waittilldone()
{
    done.lock();
    return normval;
}

void vPartObsThread::run()
{
    while(!isStopping()) {

        processing.lock();
        if(isStopping()) return;

        for(int i = pStart; i < pEnd; i++) {
            (*particles)[i].initLikelihood();
        }

        int ntoproc = std::min((int)(*stw).size(), 300);

        for(int i = pStart; i < pEnd; i++) {
            for(unsigned int j = 0; j < ntoproc; j++) {
                //if((*deltats)[j] > (*particles)[i].gettw()) break;

                //auto v = as_event<AE>((*stw)[j]);
                //auto v = is_event<AE>((*stw)[j]);
                //auto v = std::static_pointer_cast<AE>((*stw)[j]);
                //AddressEvent *v = (AddressEvent *)(*stw)[j].get();
                AE* v = read_as<AE>((*stw)[j]);

                // auto v = is_event<AE>((*stw)[j]);
                (*particles)[i].incrementalLikelihood(v->x, v->y, (*deltats)[j]);
//                if((*particles)[i].score < -20) break;
//                int l = 2 * (*particles)[i].incrementalLikelihood(v->x, v->y, (*deltats)[j]);
//                if(debugIm) {
//                    l += 128;
//                    if(l > 255) l = 255;
//                    if(l < 0) l = 0;
//                    (*debugIm)(i*4 + 0, j) = yarp::sig::PixelBgr(l, l, l);
//                    (*debugIm)(i*4 + 1, j) = yarp::sig::PixelBgr(l, l, l);
//                    (*debugIm)(i*4 + 2, j) = yarp::sig::PixelBgr(l, l, l);
//                    (*debugIm)(i*4 + 3, j) = yarp::sig::PixelBgr(l, l, l);
//                }

            }
        }

        normval = 0.0;
        for(int i = pStart; i < pEnd; i++) {
            (*particles)[i].concludeLikelihood();
            normval += (*particles)[i].getw();
        }

        done.unlock();

    }


}
