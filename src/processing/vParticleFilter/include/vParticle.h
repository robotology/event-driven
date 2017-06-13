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

    //static parameters
    int id;
    double minlikelihood;
    double inlierParameter;
    double outlierParameter;
    double variance;
    int angbuckets;
    preComputedBins *pcb;

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


    //update
    void predict(unsigned long int stamp);

    void initLikelihood();
    inline int incrementalLikelihood(int vx, int vy, int dt)
    {
        double dx = vx - x;
        double dy = vy - y;
    //    int rdx, rdy;
    //    if(dx > 0) rdx = dx + 0.5;
    //    else rdx = dx - 0.5;
    //    if(dy > 0) rdy = dy + 0.5;
    //    else rdy = dy - 0.5;

        //double sqrd = pcb->queryDistance(rdy, rdx) - r;
        double sqrd = sqrt(pow(dx, 2.0) + pow(dy, 2.0)) - r;

        if(sqrd > -inlierParameter && sqrd < inlierParameter) {

            //int a = pcb->queryBinNumber(rdy, rdx);
            int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
            if(!angdist[a]) {
                inlierCount++;
                angdist[a] = dt + 1;

                int score = inlierCount - outlierCount;
                if(score >= likelihood) {
                    likelihood = score;
                    maxtw = dt;
                }

            }



        } else if(sqrd > -outlierParameter && sqrd < 0) { //-3 < X < -5

            //int a = pcb->queryBinNumber(rdy, rdx);
            int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
            if(!negdist[a]) {
                outlierCount++;
                negdist[a] = 1;
            }

        }

        return inlierCount - outlierCount;

    }
    void concludeLikelihood();

    void updateWeightSync(double normval);

    //get
    int    getid() { return id; }
    double getx()  { return x; }
    double gety()  { return y; }
    double getr()  { return r; }
    double getw()  { return weight; }
    double getl()  { return likelihood; }
    double gettw() { return tw; }


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
