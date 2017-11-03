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

    inline double queryDistance(int dy, int dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return ds(dy, dx);
    }

    inline int queryBinNumber(double dy, double dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return (int)(bs(dy, dx) + 0.5);
    }



};

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/
class vParticle
{
private:

    //static parameters
    int id;
    double minlikelihood;
    double inlierParameter;
    double outlierParameter;
    double variance;
    int angbuckets;
    preComputedBins *pcb;
    double negscaler;

    bool constrain;
    int minx, maxx;
    int miny, maxy;
    int minr, maxr;

    //temporary parameters (on update cycle)
    double likelihood;

    double predlike;
    int    outlierCount;
    int    inlierCount;
    double maxtw;
    yarp::sig::Vector angdist;
    yarp::sig::Vector negdist;

    //state and weight
    double x;
    double y;
    double r;
    double tw;
    double weight;

    //timing
    unsigned long int stamp;

public:

    int score;


    vParticle();
    vParticle& operator=(const vParticle &rhs);

    //initialise etc.
    void initialiseParameters(int id, double minLikelihood, double outlierParam, double inlierParam, double variance, int angbuckets);
    void attachPCB(preComputedBins *pcb) { this->pcb = pcb; }

    void initialiseState(double x, double y, double r, double tw);
    void randomise(int x, int y, int r, int tw);

    void resetStamp(unsigned long int value);
    void resetWeight(double value);
    void resetRadius(double value);
    void resetArea();
    void setContraints(int minx, int maxx, int miny, int maxy, int minr, int maxr);
    void checkConstraints();


    //update
    void predict(double sigma);
    double approxatan2(double y, double x);

    void initLikelihood()
    {
        likelihood = minlikelihood;
        inlierCount = 0;
        outlierCount = 0;
        angdist.zero();
        maxtw = 0;
        score = 0;
        tw = 0;
        resetArea();
    }

    inline void incrementalLikelihood(int vx, int vy, int dt)
    {
        double dx = vx - x;
        double dy = vy - y;

        double sqrd = pcb->queryDistance((int)dy, (int)dx) - r;
        //double sqrd = sqrt(pow(dx, 2.0) + pow(dy, 2.0)) - r;

        if(sqrd > inlierParameter) {
            return;
        }

        if(sqrd > -inlierParameter) {
            //int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);

            int a = pcb->queryBinNumber((int)dy, (int)dx);

            if(!angdist[a]) {
                inlierCount++;
                angdist[a] = 1;

                score = inlierCount - (negscaler * outlierCount);
                if(score >= likelihood) {
                    likelihood = score;
                    maxtw = dt;
                }

            }

        } else {
            outlierCount++;
        }

    }

    void concludeLikelihood()
    {
        if(likelihood > minlikelihood) tw = maxtw;
        //if(likelihood == minlikelihood) likelihood *= 0.5;
        weight = likelihood * weight;
    }

    void updateWeightSync(double normval);

    //get
    inline int    getid() { return id; }
    inline double getx()  { return x; }
    inline double gety()  { return y; }
    inline double getr()  { return r; }
    inline double getw()  { return weight; }
    inline double getl()  { return likelihood; }
    inline double gettw() { return tw; }


};

/*////////////////////////////////////////////////////////////////////////////*/
// vParticleObserver
/*////////////////////////////////////////////////////////////////////////////*/
class vPartObsThread : public yarp::os::Thread
{
private:

    yarp::os::Mutex processing;
    yarp::os::Mutex done;
    int pStart;
    int pEnd;

    double normval;

    std::vector<vParticle> *particles;
    const ev::vQueue *stw;

public:

    vPartObsThread(int pStart, int pEnd);
    void setDataSources(std::vector<vParticle> *particles, const ev::vQueue *stw);
    void process();
    double waittilldone();

    void run();
};

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEFILTER
/*////////////////////////////////////////////////////////////////////////////*/

class vParticlefilter
{
private:

    //parameters
    int nparticles;
    int nthreads;
    ev::resolution res;
    bool adaptive;
    int bins;
    int seedx, seedy, seedr;
    double nRandoms;

    //data
    std::vector<vParticle> ps;
    std::vector<vParticle> ps_snap;
    std::vector<double> accum_dist;
    preComputedBins pcb;
    std::vector<vPartObsThread *> computeThreads;

    //variables
    double pwsumsq;
    int rbound_min;
    int rbound_max;

public:

    double maxlikelihood;

    vParticlefilter() {}

    void initialise(int width, int height, int nparticles,
                    int bins, bool adaptive, int nthreads, double minlikelihood,
                    double inlierThresh, double randoms);

    void setSeed(int x, int y, int r = 0);
    void resetToSeed();
    bool inbounds(vParticle &p);

    void performObservation(const vQueue &q);
    void extractTargetPosition(double &x, double &y, double &r);
    void extractTargetWindow(double &tw);
    void performResample();
    void performPrediction(double sigma);

    std::vector<vParticle> getps();

};



#endif
