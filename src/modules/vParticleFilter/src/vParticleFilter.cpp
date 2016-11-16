#include "vParticleFilter.h"
#include <cmath>
#include <limits>

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

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/

vParticle::vParticle()
{
    weight = 1.0;
    likelihood = 1.0;
    minlikelihood = 1.0;
    stamp = 0;
    nextUpdate = 0;
    fixedrate = 0;
    tw = 50000;
    leftMass = 0;
    topMass = 0;
    inlierCount = 0;
    maxlikelihood = 0;
    maxtw = 0;
    observationparameter = 0.5;
    outlierCount = 0;
    angMass = 0;
    angbuckets = 128;
    angdist.resize(angbuckets);
    negdist.resize(angbuckets);
}

void vParticle::initState(double x, double y, double r, double tw)
{
    this->x = x;
    this->y = y;
    this->r = r;
    this->tw = tw;

}


void vParticle::initState(double x, double y, double r, double vx, double vy,
              double vr)
{
    this->x = x;
    this->y = y;
    this->r = r;
}

void vParticle::initTiming(unsigned long int stamp)
{
    this->stamp = stamp;

    if(fixedrate) {
        this->nextUpdate = stamp + fixedrate;
    } else {
        this->nextUpdate = stamp + tw;
    }
}

unsigned int vParticle::getTemporalWindow()
{
    return tw;
}

void vParticle::initWeight(double weight)
{
    this->weight = weight;
}

bool vParticle::predict(unsigned long timestamp)
{

#if 1

    double dt = timestamp - this->stamp;

    //double sigmatw = 20000;
    //tw += abs(generateGaussianNoise(0, sigmatw));
    //tw += timestamp - this->stamp;
    //tw *= 1.5;
    tw += (dt*1.5);

    double sigmap = 2.0;
    x = generateGaussianNoise(x, sigmap);
    y = generateGaussianNoise(y, sigmap);
    r = generateGaussianNoise(r, sigmap * 0.2);
    if(r < 8) r = 8;
    if(r > 36) r = 36;


#elif 0

    double dt = timestamp - this->stamp;
    double halfdtsqrd = 0.5 * pow(dt, 2.0);

    double sigmap = 12 * (1e-6 * dt / 7.8125);
    double sigmav = 1e-3 * (1e-6 * dt / 7.8125);
    double sigmaa = 1e-15 * (1e-6 * dt / 7.8125);

    x = generateGaussianNoise(x + vx*dt + ax*halfdtsqrd, sigmap);
    y = generateGaussianNoise(y + vy*dt + ay*halfdtsqrd, sigmap);
    r = generateGaussianNoise(r + vr*dt + ar*halfdtsqrd, sigmap);

    vx = generateGaussianNoise(vx + ax*dt, sigmav);
    vy = generateGaussianNoise(vy + ay*dt, sigmav);
    vr = generateGaussianNoise(vr + ar*dt, sigmav);

    ax = generateGaussianNoise(ax, sigmaa);
    ay = generateGaussianNoise(ay, sigmaa);
    ar = generateGaussianNoise(ar, sigmaa);


#elif 1
    //constant velocity
    double dt = timestamp - this->stamp;
    double halfdtsqrd = 0.5 * pow(dt, 2.0);
    //double vel = sqrt(pow(vx, 2.0) + pow(vy, 2.0) + pow(vr, 2.0));
    //double acc = sqrt(pow(ax, 2.0) + pow(ay, 2.0) + pow(ar, 2.0));

    double deltax = vx * dt + ax * halfdtsqrd;
    double deltay = vy * dt + ay * halfdtsqrd;
    double deltar = vr * dt + ar * halfdtsqrd;
    double delta_mag = sqrt(pow(deltax, 2.0) + pow(deltay, 2.0) + pow(deltar, 2.0));

    double sigma = std::max(0.5 * delta_mag, 256 * (1e-6 * dt / 7.8125));
    if(!id) {
        std::cout << "Pixels: " << delta_mag << " std: ";
        std::cout << sigma << " over " << dt * 1e-6 / 7.8125 << " seconds";
        std::cout << " = " << sigma * 7.8125 * 1e6 / dt << "std / sec" <<std::endl;
    }

    double xt = generateGaussianNoise(x + deltax, sigma);
    double yt = generateGaussianNoise(y + deltay, sigma);
    double rt = generateGaussianNoise(r + deltar, sigma);

    double alpha = 0.25;
    double beta = 1-alpha;
    double vxt = beta*vx + alpha*(xt - x) / dt;
    double vyt = beta*vy + alpha*(yt - y) / dt;
    double vrt = beta*vr + alpha*(rt - r) / dt;

    ax = beta*ax + alpha*(vxt - vx) / dt;
    ay = beta*ay + alpha*(vyt - vy) / dt;
    ar = beta*ar + alpha*(vrt - vr) / dt;

    vx = vxt;
    vy = vyt;
    vr = vrt;

    x = xt;
    y = yt;
    r = rt;

#else
    //constant acceleration
    double dt = timestamp - this->stamp;
    double halfdtsqrd = 0.5 * pow(dt, 2.0);
    double sigma = 0.001e-11 * (dt * 1e-6 / 7.8125);
    x += vx * dt + ax * halfdtsqrd;
    y += vy * dt + ay * halfdtsqrd;
    r += vr * dt + ar * halfdtsqrd;

    vx += ax * dt;
    vy += ay * dt;
    vr += ar * dt;

    ax = generateGaussianNoise(ax, sigma);
    ay = generateGaussianNoise(ay, sigma);
    ar = generateGaussianNoise(ar, sigma);





#endif

    initTiming(timestamp);

    if(x < -r || y < -r || x > 127+r || y > 127+r)
        return true;
    return false;

}

bool vParticle::needsUpdating(unsigned long int stamp)
{
    return stamp > nextUpdate;
}



double vParticle::calcLikelihood(eventdriven::vQueue &events, int nparticles)
{
    double defaultweight = 0.0001;
    if(events.empty()) {
        likelihood = 0.0001;
        return 0.0001;
    }

    double inliers = 0;
    double count = 0;
    double left = 0;
    double bottom = 0;
    for(unsigned int i = 0; i < events.size(); i++) {

        eventdriven::AddressEvent * v = events[i]->getUnsafe<eventdriven::AddressEvent>();

        double sqrd = sqrt(pow(v->getX() - x, 2.0) + pow(v->getY() - y, 2.0)) - r;
        sqrd = abs(sqrd);
        if(sqrd < 2) {
            inliers++; count++;
            left += v->getX() - x;
            bottom += v->getY() - y;
        } else if(sqrd < 4) {
            count -= 2;
        }
    }

    if(inliers) {
        double d_inliers = sqrt(pow(left / inliers, 2.0) + pow(bottom / inliers, 2.0));
        count -= d_inliers;
    }

    if(count < 0) count = 0;
    count += defaultweight;

    likelihood = count;


    return likelihood;
}

void vParticle::initLikelihood()
{
    likelihood = minlikelihood;
    inlierCount = 0;
    outlierCount = 0;
    leftMass = 0;
    topMass = 0;
    angMass = 0;
    angdist.zero();
    negdist.zero();
    maxtw = 0;
}

void vParticle::incrementalLikelihood(int vx, int vy, int dt)
{
    double dx = vx - x;
    double dy = vy - y;
    if(abs(dx) > r + 2 || abs(dy) > r + 2) return;
    double sqrd = sqrt(pow(dx, 2.0) + pow(dy, 2.0)) - r;
    if(abs(sqrd) < r / 10.0) { //2.0
        //inlierCount++;
        //leftMass += dx;
        //topMass += dy;

        int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
        if(!angdist[a])
            inlierCount++;
        angdist[a] = 1;
        //if(angMass < angbuckets) return;


        //if(inlierCount > 3.0 * angbuckets / 4.0) {
            //double massval = sqrt(pow(leftMass / inlierCount, 2.0) + pow(topMass / inlierCount, 2.0));
            //double score = inlierCount - 2.0 * outlierCount - 10.0 * massval;
            //double score = inlierCount - outlierCount;
            double score = inlierCount - outlierCount;
            //if(score > likelihood && inlierCount > thresh * std::min(2.0 * M_PI * r, (double)angbuckets)) {
            if(score > likelihood && inlierCount > observationparameter * 2.0 * M_PI * r) {
                likelihood = score;
                maxtw = dt;
                topMass = inlierCount;
                leftMass = outlierCount;
            }
        //}

    } else if(sqrd < -3.0 * r / 10.0 && sqrd > -6.0 * r / 10.0) { //-3 < X < -5
    //} else if(sqrd < 0) {
        //outlierCount++;

        int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
        if(!negdist[a])
            outlierCount++;
        negdist[a] = 1;
    }



}

void vParticle::concludeLikelihood()
{
    if(likelihood > minlikelihood) {
        tw = maxtw;

        //std::cout << topMass << " - " << leftMass << " = " << likelihood << std::endl;
    }
    weight = likelihood * weight;
}

//void vParticle::initLikelihood()
//{
//    likelihood = 0;
//    inlierCount = 0;
//    leftMass = 0;
//    topMass = 0;
//    maxlikelihood = 0;
//    maxtw = 0;
//}

//void vParticle::incrementalLikelihood(int vx, int vy, int dt)
//{
//    double dx = vx - x;
//    double dy = vy - y;
//    if(abs(dx) > r + 2 || abs(dy) > r + 2) return;
//    double sqrd = sqrt(pow(dx, 2.0) + pow(dy, 2.0)) - r;
//    if(abs(sqrd) < 1.5) {
//        likelihood++;
//        inlierCount++;
//        leftMass += dx;
//        topMass += dy;
//    //} else if(sqrd < -2) {
//    } else if(sqrd > -4 && sqrd < 0) {
//        likelihood -= 2;
//    }

//    if(inlierCount > r) {
//        double massval = 10*sqrt(pow(leftMass / inlierCount, 2.0) + pow(topMass / inlierCount, 2.0));
//        if(likelihood - massval > maxlikelihood) {
//            maxlikelihood = likelihood - massval;
//            maxtw = dt;
//        }
//    }

////    if(inlierCount > r) {
////        if(likelihood > maxlikelihood) {
////            maxlikelihood = likelihood;
////            maxtw = dt;
////        }
////    }
//}

//void vParticle::concludeLikelihood()
//{
//    likelihood = maxlikelihood;

////    if(inlierCount)
////        likelihood -= 1.0 * sqrt(pow(leftMass / inlierCount, 2.0) + pow(topMass / inlierCount, 2.0));

//    //if(likelihood <= 0 || inlierCount < 2.0*r) {
//    if(likelihood <= r) {
//        likelihood = 1.0;
//    } else {
//        tw = maxtw;
//    }

//    weight = likelihood * weight;
//}

void vParticle::updateWeight(double l, double n)
{
    weight = l * weight / n;
    //weight = l * weight / (l * weight + (n-weight)*n);
    //weight = l * weight / (l * weight + (1.0 - 1.0/n)*n);
}

void vParticle::updateWeight2(double likelihood, double pwsumsq)
{
    //weight = likelihood * weight / (likelihood * weight + pwsumsq - (weight*weight));


    weight = likelihood / (likelihood + pwsumsq/weight - weight);

}
void vParticle::updateWeightSync(double normval)
{
    weight = weight / normval;
}

void vParticle::resample(double w, unsigned long int t)
{

    //initState(rand()%128, rand()%128, 10+rand()%26, 10000 + rand()%200000);
    initState(rand()%128, rand()%128, 10+rand()%26, 100);
    initWeight(w);
    initTiming(t);
    return;
//    int s1 = rand() % 2 > 0 ? 1 : -1;
//    int s2 = rand() % 2 > 0 ? 1 : -1;

//    initState(rand()%128, rand()%128, 12+rand()%8,
//              7.8125*s1*(0.5e-6 + ((double)rand() / RAND_MAX) * 2.5e-6),
//              7.8125*s2*(0.5e-6 + ((double)rand() / RAND_MAX) * 2.5e-6), 0);
//    initTiming(t);
    //initWeight(w);

}

void vParticle::resample(const vParticle &seeder, double w, unsigned long int t)
{

    x = seeder.x;
    y = seeder.y;
    r = seeder.r;
//    vx = seeder.vx;
//    vy = seeder.vy;
//    vy = seeder.vy;
//    ax = seeder.ax;
//    ay = seeder.ay;
//    ar = seeder.ar;

    stamp = seeder.stamp;
    weight = seeder.weight;

}
