
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
    double leftMass;
    double topMass;
    int    inlierCount;
    double maxlikelihood;
    double maxtw;
    //bool doneLikelihood;

    //state - this should be a yarp::sig::vector
    double x;
    double y;
    double r;
    //double vx;
    //double vy;
    //double vr;
    //double ax;
    //double ay;
    //double ar;
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

    void resample(double w, unsigned long int t);
    void resample(const vParticle &seeder, double w, unsigned long int t);


    bool predict(unsigned long int stamp);
    double calcLikelihood(ev::vQueue &events, int nparticles);
    void initLikelihood();
    void incrementalLikelihood(int vx, int vy, int dt);
    void concludeLikelihood();


    void updateWeight(double l, double n);
    void updateWeight2(double likelihood, double pwsumsq);
    void updateWeightSync(double normval);

    double getx() { return x; }
    double gety() { return y; }
    double getr() { return r; }
    //double getvx() { return vx; }
    //double getvy() { return vy; }
    //double getvr() { return vr; }
    //double getax() { return ax; }
    //double getay() { return ay; }
    //double getar() { return ar; }
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
