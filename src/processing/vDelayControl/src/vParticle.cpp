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

#include "vParticle.h"
#include <cmath>
#include <limits>
#include <algorithm>

using ev::event;
using ev::AddressEvent;

double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, ev::vQueue &q,
                int offsetx) {

    if(q.empty()) return;

    //draw oldest first
    for(int i = (int)q.size()-1; i >= 0; i--) {
        double p = (double)i / (double)q.size();
        auto v = is_event<AE>(q[i]);
        image(v->x + offsetx, v->y) =
                yarp::sig::PixelBgr(255 * (1-p), 0, 255);
    }
}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy,
                int cr, int id)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8)
                continue;
            int px = cx + x; int py = cy + y;
            if(py<0 || py>(int)image.height()-1 || px<0 || px>(int)image.width()-1)
                continue;
            switch(id) {
            case(0): //green
                image(px, py) = yarp::sig::PixelBgr(0, 255, 0);
                break;
            case(1): //blue
                image(px, py) = yarp::sig::PixelBgr(0, 0, 255);
                break;
            case(2): //red
                image(px, py) = yarp::sig::PixelBgr(255, 0, 0);
                break;
            default:
                image(px, py) = yarp::sig::PixelBgr(255, 255, 0);
                break;

            }

        }
    }

}

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<vParticle> &indexedlist)
{

    double sum = 0;
    std::vector<double> weights;
    for(unsigned int i = 0; i < indexedlist.size(); i++) {
        weights.push_back(indexedlist[i].getw());
        sum += weights.back();
    }

    std::sort(weights.begin(), weights.end());


    image.resize(indexedlist.size(), 100);
    image.zero();
    for(unsigned int i = 0; i < weights.size(); i++) {
        image(weights.size() - 1 -  i, 99 - weights[i]*100) = yarp::sig::PixelBgr(255, 255, 255);
    }
}

double vParticle::approxatan2(double y, double x) {

    double ax = std::abs(x); double ay = std::abs(y);
    double a = std::min (ax, ay) / std::max (ax, ay);
    //double s = pow(a, 2.0);
    //double r = ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a;

    double r = a * (M_PI_4 - (a - 1) * 0.273318560862312);

    if(ay > ax)
        r = 1.57079637 - r;

    if(x < 0)
        r = 3.14159274 - r;
    if(y < 0)
        r = -r;

    return r;

}

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/

vParticle::vParticle()
{
    weight = 1.0;
    likelihood = 1.0;
    predlike = 1.0;
    minlikelihood = 20.0;
    inlierParameter = 1.5;
    variance = 0.5;
    nw = 0;
    inlierCount = 0;
    outlierCount = 0;
    pcb = 0;
    angbuckets = 128;
    negativeBias = 2.0;
    angdist.resize(angbuckets);
    negdist.resize(angbuckets);
    constrain = false;
}

void vParticle::initialiseParameters(int id, double minLikelihood,
                                     double negativeBias, double inlierParam,
                                     double variance, int angbuckets)
{
    this->id = id;
    this->negativeBias = negativeBias;
    this->inlierParameter = inlierParam;
    this->variance = variance;
    this->angbuckets = angbuckets;
    angdist.resize(angbuckets);
    negdist.resize(angbuckets);

    updateMinLikelihood(minLikelihood);
}

void vParticle::updateMinLikelihood(double value)
{
    minlikelihood = value * angbuckets;
}

vParticle& vParticle::operator=(const vParticle &rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->r = rhs.r;
    this->weight = rhs.weight;
    return *this;
}

void vParticle::initialiseState(double x, double y, double r)
{
    this->x = x;
    this->y = y;
    this->r = r;
}

void vParticle::randomise(int x, int y, int r)
{
    initialiseState(rand()%x, rand()%y, rand()%r);
}

void vParticle::resetWeight(double value)
{
    this->weight = value;
}

void vParticle::resetRadius(double value)
{
    this->r = value;
    //resetArea();
}

void vParticle::setNegativeBias(double value)
{
    negativeBias = value;
}

void vParticle::setInlierParameter(double value)
{
    inlierParameter = value;
}

void vParticle::resetArea()
{
    negativeScaler = negativeBias * angbuckets / (M_PI * r * r);
}

void vParticle::predict(double sigma)
{
    //tw += 12500;
    x = generateGaussianNoise(x, sigma);
    y = generateGaussianNoise(y, sigma);
    r = generateGaussianNoise(r, sigma * 0.2);

    if(constrain) checkConstraints();
}

void vParticle::setContraints(int minx, int maxx, int miny, int maxy, int minr, int maxr)
{
    this->minx = minx;
    this->maxx = maxx;
    this->miny = miny;
    this->maxy = maxy;
    this->minr = minr;
    this->maxr = maxr;
    constrain = true;
}
void vParticle::checkConstraints()
{
    if(x < minx) x = minx;
    if(x > maxx) x = maxx;
    if(y < miny) y = miny;
    if(y > maxy) y = maxy;
    if(r < minr) r = minr;
    if(r > maxr) r = maxr;
}



void vParticle::updateWeightSync(double normval)
{
    weight = weight / normval;
}

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEFILTER
/*////////////////////////////////////////////////////////////////////////////*/


void vParticlefilter::initialise(int width, int height, int nparticles,
                                 int bins, bool adaptive, int nthreads,
                                 double minlikelihood, double inlierThresh,
                                 double randoms, double negativeBias)
{
    res.width = width;
    res.height = height;
    this->nparticles = nparticles;
    this->bins = bins;
    this->adaptive = adaptive;
    this->nthreads = nthreads;
    this->nRandoms = randoms + 1.0;
    rbound_min = res.width/18;
    rbound_max = res.width/5;
    pcb.configure(res.height, res.width, rbound_max, bins);
    setSeed(res.width/2.0, res.height/2.0);

    ps.clear();
    ps_snap.clear();
    accum_dist.resize(this->nparticles);

    if(this->nthreads > 1) {
        for(int i = 0; i < this->nthreads; i++) {
            int pStart = i * (this->nparticles / this->nthreads);
            int pEnd = (i+1) * (this->nparticles / this->nthreads);
            if(i == this->nthreads - 1)
                pEnd = this->nparticles;

            yInfo() << "Thread" << i << "=" << pStart << "->" << pEnd-1;
            computeThreads.push_back(new vPartObsThread(pStart, pEnd));
            computeThreads[i]->start();
        }
    }


    vParticle p;
    p.attachPCB(&pcb);
    p.resetWeight(1.0/nparticles);
    p.setContraints(0, res.width, 0, res.height, rbound_min, rbound_max);
    for(int i = 0; i < this->nparticles; i++) {
        p.initialiseParameters(i, minlikelihood, negativeBias, inlierThresh, 0, bins);
        ps.push_back(p);
        ps_snap.push_back(p);
    }

    resetToSeed();
}

void vParticlefilter::setSeed(int x, int y, int r)
{
    seedx = x; seedy = y; seedr = r;
}

void vParticlefilter::resetToSeed()
{
    if(seedr) {
        for(int i = 0; i < nparticles; i++) {
            ps[i].initialiseState(seedx, seedy, seedr);
        }
    } else {
        for(int i = 0; i < nparticles; i++) {
            ps[i].initialiseState(seedx, seedy,
                                  rbound_min + (rbound_max - rbound_min) *
                                  ((double)rand()/RAND_MAX));
        }
    }
}

void vParticlefilter::setMinLikelihood(double value)
{
    for(int i = 0; i < nparticles; i++)
        ps[i].updateMinLikelihood(value);
}

void vParticlefilter::setInlierParameter(double value)
{
    for(int i = 0; i < nparticles; i++)
        ps[i].setInlierParameter(value);
}

void vParticlefilter::setNegativeBias(double value)
{
    for(int i = 0; i < nparticles; i++)
        ps[i].setNegativeBias(value);
}

void vParticlefilter::setAdaptive(bool value)
{
    adaptive = value;
}

void vParticlefilter::performObservation(const vQueue &q)
{
    double normval = 0.0;
    if(nthreads == 1) {

        //START WITHOUT THREAD
        for(int i = 0; i < nparticles; i++) {
            ps[i].initLikelihood(q.size());
        }

        for(int i = 0; i < nparticles; i++) {
            for(int j = 0; j < (int)q.size(); j++) {
                AE* v = read_as<AE>(q[j]);
                ps[i].incrementalLikelihood(v->x, v->y, j);
            }
        }

        for(int i = 0; i < nparticles; i++) {
            ps[i].concludeLikelihood();
            normval += ps[i].getw();
        }
    } else {

        //START MULTI-THREAD
        for(int k = 0; k < nthreads; k++) {
            computeThreads[k]->setDataSources(&ps, &q);
            computeThreads[k]->process();
        }

        for(int k = 0; k < nthreads; k++) {
            normval += computeThreads[k]->waittilldone();
        }
    }

    pwsumsq = 0;
    maxlikelihood = 0;
    for(int i = 0; i < nparticles; i ++) {
        ps[i].updateWeightSync(normval);
        pwsumsq += pow(ps[i].getw(), 2.0);
        maxlikelihood = std::max(maxlikelihood, ps[i].getl());
    }

}

void vParticlefilter::extractTargetPosition(double &x, double &y, double &r)
{
    x = 0; y = 0; r = 0;

    for(int i = 0; i < nparticles; i++) {
        double w = ps[i].getw();
        x += ps[i].getx() * w;
        y += ps[i].gety() * w;
        r += ps[i].getr() * w;
    }
}

void vParticlefilter::extractTargetWindow(double &tw)
{
    tw = 0;

    for(int i = 0; i < nparticles; i++) {
        double w = ps[i].getw();
        tw += ps[i].getnw() * w;

    }

}

void vParticlefilter::performResample()
{
    if(!adaptive || pwsumsq * nparticles > 2.0) {
        //initialise for the resample
        double accum = 0;
        for(int i = 0; i < nparticles; i++) {
            ps_snap[i] = ps[i];
            accum += ps[i].getw();
            accum_dist[i] = accum;
        }

        //perform the resample
        for(int i = 0; i < nparticles; i++) {
            double rn = nRandoms * (double)rand() / RAND_MAX;
            if(rn > 1.0)
                ps[i].randomise(res.width, res.height, rbound_max);
            else {
                int j = 0;
                for(j = 0; j < nparticles; j++)
                    if(accum_dist[j] > rn) break;
                ps[i] = ps_snap[j];
            }
        }
    }

}

void vParticlefilter::performPrediction(double sigma)
{
    for(int i = 0; i < nparticles; i++)
        ps[i].predict(sigma);
}

std::vector<vParticle> vParticlefilter::getps()
{
    return ps;
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
                                    const vQueue *stw)
{
    this->particles = particles;
    this->stw = stw;
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

        int nw = (*stw).size();

        for(int i = pStart; i < pEnd; i++) {
            (*particles)[i].initLikelihood(nw);
        }

        for(int i = pStart; i < pEnd; i++) {
            for(int j = 0; j < nw; j++) {
                AE* v = read_as<AE>((*stw)[j]);
                (*particles)[i].incrementalLikelihood(v->x, v->y, j);
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
