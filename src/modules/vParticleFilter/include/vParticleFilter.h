
#ifndef __V_PARTICLEFILTER__
#define __V_PARTICLEFILTER__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/
class vParticle
{
private:

    //id
    int id;

    //weight
    double weight;
    double likelihood;
    double observationparameter;
    double leftMass;
    double topMass;
    double angMass;
    int    outlierCount;
    int    inlierCount;
    double maxlikelihood;
    double minlikelihood;
    double maxtw;
    int angbuckets;
    yarp::sig::Vector angdist;
    yarp::sig::Vector negdist;

    //state - this should be a yarp::sig::vector
    double x;
    double y;
    double r;
    double tw;

    //timing
    unsigned long int stamp;
    unsigned long int nextUpdate;
    unsigned int fixedrate;

public:

    vParticle();

    void setid(int id) { this->id = id; }
    int getid() { return id; }

    void initState(double x, double y, double r,
                   double vx, double vy, double vr);
    void initState(double x, double y, double r, double tw);


    void initTiming(unsigned long int stamp);
    void initWeight(double weight);


    unsigned int getTemporalWindow();

    void setRate(unsigned int rate) { fixedrate = rate; }
    void setMinLikelihood(double minlikelihood) { this->minlikelihood = minlikelihood; }
    void setObservationParameter(double value) { this->observationparameter = value; }

    void resample(double w, unsigned long int t);
    void resample(const vParticle &seeder, double w, unsigned long int t);


    bool predict(unsigned long int stamp);
    double calcLikelihood(eventdriven::vQueue &events, int nparticles);
    void initLikelihood();
    void incrementalLikelihood(int vx, int vy, int dt);
    void concludeLikelihood();


    void updateWeight(double l, double n);
    void updateWeight2(double likelihood, double pwsumsq);
    void updateWeightSync(double normval);

    double getx() { return x; }
    double gety() { return y; }
    double getr() { return r; }
    double getw() { return weight; }
    double getl() { return likelihood; }
    double gettw() { return tw; }
    unsigned long getUpdateTime() { return nextUpdate; }
    unsigned long getStamp() { return stamp; }
    bool needsUpdating(unsigned long int stamp);

    bool operator<(const vParticle &p) const
                 {return this->nextUpdate > p.nextUpdate; }
    bool operator>(const vParticle &p) const
                 {return this->nextUpdate < p.nextUpdate; }

};


#endif
