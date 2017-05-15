#ifndef __VPARTICLE__
#define __VPARTICLE__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>

using namespace ev;

class vParticle;

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, ev::vQueue &q, int currenttime, double tw = 0, bool flip = false);

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id = 0);

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<vParticle> &indexedlist);

class preComputedBins;

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/
class vParticle
{
private:

    double minlikelihood;
    double inlierParameter;
    double outlierParameter;
    double variance;

    //id
    int id;

    //weight
    double weight;
    double likelihood;
    int    outlierCount;
    int    inlierCount;
    double maxtw;
    int angbuckets;
    yarp::sig::Vector angdist;
    yarp::sig::Vector negdist;
    preComputedBins *pcb;


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

    void setPCB(preComputedBins *pcb) { this->pcb = pcb; }

    void setRate(unsigned int rate) { fixedrate = rate; }
    void setMinLikelihood(double minlikelihood) { this->minlikelihood = minlikelihood; }
    void setOutlierParameter(double value) { this->outlierParameter = value; }
    void setInlierParameter(double value) {this->inlierParameter = value; }
    void setVariance(double value) { this->variance = value; }

    void resample(double w, unsigned long int t, int x, int y, int r, int tw);
    void resample(const vParticle &seeder, double w, unsigned long int t);


    bool predict(unsigned long int stamp);
    double calcLikelihood(ev::vQueue &events, int nparticles);
    void initLikelihood();
    int incrementalLikelihood(int vx, int vy, int dt);
    void concludeLikelihood();


    void updateWeight(double l, double n);
    void updateWeight2(double likelihood, double pwsumsq);
    void updateWeightSync(double normval);

    double dtavg;
    double dtvar;

    double getx() { return x; }
    double gety() { return y; }
    double getr() { return r; }
    double getw() { return weight; }
    double getl() { return likelihood; }
    double gettw() { return tw; }
    unsigned long getUpdateTime() { return nextUpdate; }
    unsigned long getStamp() { return stamp; }
    void setr(double value) { this->r = value; }
    bool needsUpdating(unsigned long int stamp);

    bool operator<(const vParticle &p) const
        { return this->nextUpdate > p.nextUpdate; }
    bool operator>(const vParticle &p) const
        { return this->nextUpdate < p.nextUpdate; }

};

class preComputedBins
{

private:

    yarp::sig::Matrix ds;
    yarp::sig::Matrix bs;
    int rows;
    int cols;
    int offsetx;
    int offsety;

public:

    preComputedBins()
    {
        rows = 0;
        cols = 0;
        offsetx = 0;
        offsety = 0;
    }

    void configure(int height, int width, double maxrad, int nBins)
    {
        rows = (height + maxrad) * 2 + 1;
        cols = (width + maxrad) * 2 + 1;
        offsety = rows/2;
        offsetx = cols/2;

        ds.resize(rows, cols);
        bs.resize(rows, cols);
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {

                int dy = i - offsety;
                int dx = j - offsetx;

                ds(i, j) = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
                bs(i, j) = (nBins-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);

            }
        }
    }

    double queryDistance(int dy, int dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return ds(dy, dx);
    }

    int queryBinNumber(double dy, double dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return (int)(bs(dy, dx) + 0.5);
    }



};


#endif
