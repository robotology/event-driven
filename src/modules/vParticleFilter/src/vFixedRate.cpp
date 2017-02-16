#include "vFixedRate.h"

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
//particle reader (callback)
/*////////////////////////////////////////////////////////////////////////////*/
vParticleReader::vParticleReader()
{

    strict = false;
    pmax.initWeight(0.0);
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
}

void vParticleReader::initialise(unsigned int width , unsigned int height,
                                 unsigned int nParticles, unsigned int rate,
                                 double nRands, bool adaptive, double pVariance)
{
    //parameters
    res.width = width;
    res.height = height;

    surfaceLeft = ev::temporalSurface(width, height);

    nparticles = nParticles;
    this->nRandomise = 1.0 + nRands;
    this->adaptive = adaptive;
    this->rate = rate;

    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);

    //initialise the particles
    vParticle p;
    p.setRate(rate);
    p.initWeight(1.0/nparticles);
    p.setInlierParameter(obsInlier);
    p.setOutlierParameter(obsOutlier);
    p.setMinLikelihood(obsThresh);
    p.setVariance(pVariance);

    for(int i = 0; i < nparticles; i++) {
        p.setid(i);
        p.resample(1.0/nparticles, 0, res.width, res.height, 30.0);
        if(seedr)
            p.initState(seedx, seedy, seedr, 100);
        p.initWeight(1.0/nparticles);
        sortedlist.push(p);
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

    return true;
}

/******************************************************************************/
void vParticleReader::close()
{
    //close ports
    scopeOut.close();
    debugOut.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

}

/******************************************************************************/
void vParticleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    scopeOut.interrupt();
    debugOut.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

bool vParticleReader::inbounds(vParticle &p)
{
    int r = p.getr();
    if(r < 20) {
        p.setr(20);
        r = 20;
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
    ev::vQueue q = inputBottle.get<AddressEvent>();
    //q.sort(true);

    ev::vQueue stw;
    unsigned long t = 0;

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        if(!(*qi)->getChannel()) continue;

        surfaceLeft.addEvent(*qi);

        t = unwrap((*qi)->getStamp());

        if(!indexedlist[0].needsUpdating(t)) continue;

        //RESAMPLE
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            std::vector<vParticle> indexedSnap = indexedlist;
            for(int i = 0; i < nparticles; i++) {
                double rn = this->nRandomise * pwsum * (double)rand() / RAND_MAX;
                if(rn > pwsum)
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

        //PREDICT
        unsigned int maxtw = 0;
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].predict(t);
            if(!inbounds(indexedlist[i])) {
                indexedlist[i].resample(1.0/nparticles, t, res.width, res.height, 30.0);
            }
            if(indexedlist[i].gettw() > maxtw)
                maxtw = indexedlist[i].gettw();
        }

        //OBSERVE
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].initLikelihood();
        }

        stw = surfaceLeft.getSurf_Tlim(maxtw);
        unsigned int ctime = (*qi)->getStamp();
        for(unsigned int i = 0; i < stw.size(); i++) {
            //calc dt
            double dt = ctime - stw[i]->getStamp();
            if(dt < 0)
                dt += ev::vtsHelper::maxStamp();
            event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(stw[i]);
            int x = v->getX();
            int y = v->getY();
            for(int i = 0; i < nparticles; i++) {
                if(dt < indexedlist[i].gettw())
                    indexedlist[i].incrementalLikelihood(x, y, dt);
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

        indexedlist[0].initTiming(t);

        yarp::os::Bottle &trackBottle = resultOut.prepare();
        trackBottle.clear();
        resultOut.setEnvelope(st);
        trackBottle.addInt(t);
        trackBottle.addDouble(avgx);
        trackBottle.addDouble(avgy);
        trackBottle.addDouble(avgr);
        trackBottle.addDouble(avgtw);

        resultOut.writeStrict();

    }

    //yarp::os::Bottle &sob = scopeOut.prepare();
    //sob.clear();
    //sob.addDouble(t);
    //if(q.size())
        //sob.addDouble(q.back()->getStamp());
    //scopeOut.write();


    if(debugOut.getOutputCount()) {
        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
        image.resize(res.width, res.height);
        image.zero();
        stw = surfaceLeft.getSurf_Tlim(avgtw);

        for(unsigned int i = 0; i < indexedlist.size(); i++) {

            int py = indexedlist[i].gety();
            int px = indexedlist[i].getx();

            if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
            //pcol = yarp::sig::PixelBgr(255*indexedlist[i].getw()/pmax.getw(), 255*indexedlist[i].getw()/pmax.getw(), 255);
            image(px, py) = yarp::sig::PixelBgr(255, 255, 255);
            //drawcircle(image, indexedlist[i].getx(), indexedlist[i].gety(), indexedlist[i].getr(), indexedlist[i].getid());
        }
        drawEvents(image, stw);
        drawcircle(image, avgx, avgy, avgr, 1);
        debugOut.write();
    }


}
