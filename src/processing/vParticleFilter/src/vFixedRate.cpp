#include "vFixedRate.h"

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
//particle reader (callback)
/*////////////////////////////////////////////////////////////////////////////*/
vParticleReader::vParticleReader()
{

    strict = false;
    pmax.resetWeight(0.0);
    srand(yarp::os::Time::now());

    avgx = 64;
    avgy = 64;
    avgr = 20;
    nparticles = 50;
    pwsum = 1.0;
    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);
    rate = 1000;
    seedx = 0; seedy = 0; seedr = 0;

    obsThresh = 30.0;
    obsInlier = 1.5;
    obsOutlier = 3.0;

    rbound_max = 50;
    rbound_min = 10;

}

void vParticleReader::initialise(unsigned int width , unsigned int height,
                                 unsigned int nParticles, unsigned int rate,
                                 double nRands, bool adaptive, double pVariance, int camera, bool useROI)
{
    //parameters
    res.width = width;
    res.height = height;

    this->camera = camera;
    this->useroi = useROI;

    surfaceLeft = ev::temporalSurface(width, height);

    rbound_min = res.width/25;
    rbound_max = res.width/6;

    pcb.configure(res.height, res.width, rbound_max, 128);

    nparticles = nParticles;
    this->nRandomise = 1.0 + nRands;
    this->adaptive = adaptive;
    this->rate = rate;

    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);

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

        indexedlist.push_back(p);
    }



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
    if(!yarp::os::BufferedPort<ev::vBottle>::open(name + "/vBottle:i"))
        return false;
    if(!scopeOut.open(name + "/scope:o"))
        return false;
    if(!debugOut.open(name + "/debug:o"))
        return false;
    if(!resultOut.open(name + "/result:o"))
        return false;
    if(!vBottleOut.open(name + "/vBottle:o"))
        return false;

    return true;
}

/******************************************************************************/
void vParticleReader::close()
{
    //close ports
    scopeOut.close();
    debugOut.close();
    vBottleOut.close();
    resultOut.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

}

/******************************************************************************/
void vParticleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    scopeOut.interrupt();
    debugOut.interrupt();
    vBottleOut.interrupt();
    resultOut.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

bool vParticleReader::inbounds(vParticle &p)
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


/******************************************************************************/
void vParticleReader::onRead(ev::vBottle &inputBottle)
{

    yarp::os::Stamp st;
    getEnvelope(st);
    if(st.getCount() != pstamp.getCount() +1) {
        std::cout << "Lost Bottle" << std::endl;
    }
    pstamp = st;

    //create event queue
    vQueue q = inputBottle.get<AE>();
    //q.sort(true);

    ev::vQueue stw;
    unsigned long pt = 0;
    unsigned long t = 0;

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        if((*qi)->getChannel() != camera) continue;

        surfaceLeft.addEvent(*qi);

        t = unwrap((*qi)->stamp);
        if(t - pt < rate) continue;
        pt = t;

        //if(!indexedlist[0].needsUpdating(t)) continue;

        //RESAMPLE
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            std::vector<vParticle> indexedSnap = indexedlist;
            for(int i = 0; i < nparticles; i++) {
                double rn = this->nRandomise * pwsum * (double)rand() / RAND_MAX;
                if(rn > pwsum)
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

        //PREDICT
        unsigned int maxtw = 0;
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].predict(t);
            if(!inbounds(indexedlist[i])) {
                indexedlist[i].randomise(res.width, res.height, 30.0, avgtw);
            }
            if(indexedlist[i].gettw() > maxtw)
                maxtw = indexedlist[i].gettw();
        }

        //OBSERVE
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].initLikelihood();
        }

        stw = surfaceLeft.getSurf_Tlim(maxtw);
        unsigned int ctime = (*qi)->stamp;
        for(unsigned int i = 0; i < stw.size(); i++) {
            //calc dt
            double dt = ctime - stw[i]->stamp;
            if(dt < 0) dt += ev::vtsHelper::max_stamp;
            auto v = is_event<AE>(stw[i]);
            for(int i = 0; i < nparticles; i++) {
                if(dt < indexedlist[i].gettw())
                    indexedlist[i].incrementalLikelihood(v->x, v->y, dt);
            }

        }

        //NORMALISE
        double normval = 0.0;
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].concludeLikelihood();
            normval += indexedlist[i].getw();
        }


        //FIND THE AVERAGE POSITION
        pwsum = 0;
        pwsumsq = 0;
        avgx = 0;
        avgy = 0;
        avgr = 0;
        avgtw = 0;

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
        }

        //indexedlist[0].resetStamp(t);

        if(vBottleOut.getOutputCount()) {
            ev::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
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
            vBottleOut.setEnvelope(st);
            vBottleOut.write();
        }

        if(resultOut.getOutputCount()) {
            yarp::os::Bottle &trackBottle = resultOut.prepare();
            trackBottle.clear();
            resultOut.setEnvelope(st);
            trackBottle.addInt(t);
            trackBottle.addDouble(avgx);
            trackBottle.addDouble(avgy);
            trackBottle.addDouble(avgr);
            trackBottle.addDouble(avgtw);
            resultOut.setEnvelope(st);
            resultOut.writeStrict();
        }

    }

    if(scopeOut.getOutputCount()) {

        yarp::os::Bottle &sob = scopeOut.prepare();
        sob.clear();

        double dt = q.back()->stamp - q.front()->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        sob.addDouble(dt);
        sob.addDouble(q.size());
        scopeOut.setEnvelope(st);
        scopeOut.write();
    }


    if(debugOut.getOutputCount()) {
        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
        image.resize(res.width, res.height);
        image.zero();
        //stw = surfaceLeft.getSurf_Tlim(avgtw);

        for(unsigned int i = 0; i < indexedlist.size(); i++) {

            int py = indexedlist[i].gety();
            int px = indexedlist[i].getx();

            if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
            //pcol = yarp::sig::PixelBgr(255*indexedlist[i].getw()/pmax.getw(), 255*indexedlist[i].getw()/pmax.getw(), 255);
            //image(res.width - px - 1, res.height - py - 1) = yarp::sig::PixelBgr(255, 255, 255);
            image(px, py) = yarp::sig::PixelBgr(255, 255, 255);
            //drawcircle(image, indexedlist[i].getx(), indexedlist[i].gety(), indexedlist[i].getr(), indexedlist[i].getid());
        }
        drawEvents(image, stw, avgtw, false);
        //drawcircle(image, res.width - 1 - avgx, res.height - 1 - avgy, avgr, 1);
        drawcircle(image, avgx, avgy, avgr, 1);
        debugOut.setEnvelope(st);
        debugOut.write();
    }


}
