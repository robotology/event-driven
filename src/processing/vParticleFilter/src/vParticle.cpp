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

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, ev::vQueue &q, int currenttime, double tw, bool flip) {

    if(q.empty()) return;

    for(unsigned int i = 0; i < q.size(); i++) {
        if(tw) {
            double dt = currenttime - q[i]->stamp;
            if(dt < 0) dt += ev::vtsHelper::max_stamp;
            if(dt > tw) break;
        }

        auto v = is_event<AE>(q[i]);
        if(flip)
            image(image.width() - 1 - v->x, image.height() - 1 - v->y) = yarp::sig::PixelBgr(0, 255, 0);
        else
            image(v->x, v->y) = yarp::sig::PixelBgr(0, 255, 0);

    }
}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8) continue;
            int px = cx + x; int py = cy + y;
            if(py < 0 || py > image.height()-1 || px < 0 || px > image.width()-1) continue;
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

double approxatan2(double y, double x) {

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
    minlikelihood = 20.0;
    inlierParameter = 1.5;
    outlierParameter = 3.0;
    variance = 0.5;
    stamp = 0;
    tw = 0;
    inlierCount = 0;
    maxtw = 0;
    outlierCount = 0;
    pcb = 0;
    angbuckets = 128;
    angdist.resize(angbuckets);
    negdist.resize(angbuckets);
}

void vParticle::initialiseParameters(int id, double minLikelihood,
                                     double outlierParam, double inlierParam,
                                     double variance, int angbuckets)
{
    this->id = id;
    this->minlikelihood = minLikelihood;
    this->outlierParameter = outlierParam;
    this->inlierParameter = inlierParam;
    this->variance = variance;
    this->angbuckets = angbuckets;
    angdist.resize(angbuckets);
    negdist.resize(angbuckets);
}

vParticle& vParticle::operator=(const vParticle &rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->r = rhs.r;
    this->tw = rhs.tw;
    this->weight = rhs.weight;
    this->stamp = rhs.stamp;

}

void vParticle::initialiseState(double x, double y, double r, double tw)
{
    this->x = x;
    this->y = y;
    this->r = r;
    this->tw = tw;
}

void vParticle::randomise(int x, int y, int r, int tw)
{
    initialiseState(rand()%x, rand()%y, rand()%r, rand()%tw);
}

void vParticle::resetStamp(unsigned long int value)
{
    stamp = value;
}

void vParticle::resetWeight(double value)
{
    this->weight = weight;
}

void vParticle::resetRadius(double value)
{
    this->r = value;
}

void vParticle::predict(unsigned long timestamp)
{
    double dt = 0;
    if(stamp) dt = timestamp - this->stamp;
    stamp = timestamp;

    tw += dt;
    tw = std::max(tw, 50000.0);

    x = generateGaussianNoise(x, variance);
    y = generateGaussianNoise(y, variance);
    r = generateGaussianNoise(r, variance * 0.4);

}

void vParticle::initLikelihood()
{
    likelihood = minlikelihood;
    inlierCount = 0;
    outlierCount = 0;
    angdist.zero();
    negdist.zero();
    maxtw = 0;
}

int vParticle::incrementalLikelihood(int vx, int vy, int dt)
{
    double dx = vx - x;
    double dy = vy - y;
    int rdx, rdy;
    if(dx > 0) rdx = dx + 0.5;
    else rdx = dx - 0.5;
    if(dy > 0) rdy = dy + 0.5;
    else rdy = dy - 0.5;

    double sqrd = pcb->queryDistance(rdy, rdx) - r;
    //double sqrd = sqrt(pow(dx, 2.0) + pow(dy, 2.0)) - r;

    if(sqrd > -inlierParameter && sqrd < inlierParameter) {

        int a = pcb->queryBinNumber(rdy, rdx);
        //int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
        if(!angdist[a]) {
            inlierCount++;
            angdist[a] = dt + 1;
        }

    } else if(sqrd > -outlierParameter && sqrd < 0) { //-3 < X < -5

        int a = pcb->queryBinNumber(rdy, rdx);
        //int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
        if(!negdist[a]) {
            outlierCount++;
            negdist[a] = 1;
        }

    }

    int score = inlierCount - outlierCount;
    if(score >= likelihood) {
        likelihood = score;
        maxtw = dt;
    }


    return inlierCount - outlierCount;

}

void vParticle::concludeLikelihood()
{
    double dtavg = 0;
    double dtvar = 0;
    int n = 0;

    for(unsigned int i = 0; i < angdist.size(); i++) {
        if(angdist[i] == 0) continue;
        dtavg += angdist[i];
        n++;
    }
    if(n > 10) {
        dtavg /= n;
        for(unsigned int i = 0; i < angdist.size(); i++) {
            if(angdist[i] == 0) continue;
            dtvar += (dtavg - angdist[i]) * (dtavg - angdist[i]);
        }
        dtvar /= n;
        dtvar = 0.000001 + 1.0 / sqrt(dtvar * 2.0 * M_PI);
    } else {
        dtvar = 0.000001;
    }

    if(likelihood > minlikelihood) tw = maxtw;
    weight = likelihood * weight * dtvar;

}

void vParticle::updateWeightSync(double normval)
{
    weight = weight / normval;
}



