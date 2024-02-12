#include <event-driven/algs/flow.h>
#include <vector>

namespace ev {

const std::vector< std::vector<cv::Point> > pwtripletvelocity::is = {{{0,0},{1,1}},
                                                                    {{0,2},{1,2}}, 
                                                                    {{0,4},{1,3}}, 
                                                                    {{2,4},{2,3}}, 
                                                                    {{4,4},{3,3}}, 
                                                                    {{4,2},{3,2}}, 
                                                                    {{4,0},{3,1}}, 
                                                                    {{2,0},{2,1}}};

const std::vector<cv::Point2d> pwtripletvelocity::vs ={{1,  1}, 
                                                {1,  0}, 
                                                {1, -1}, 
                                                {0, -1}, 
                                                {-1,-1}, 
                                                {-1, 0}, 
                                                {-1, 1}, 
                                                {0,  1}};

pwtripletvelocity::wjv pwtripletvelocity::point_velocity(const cv::Mat &local_sae)
{

    wjv wvel = {{0,0}, 0};

    for(size_t i = 0; i < is.size(); i++) 
    {
        const double &t0 = local_sae.at<double>(2, 2);
        const double &t1 = local_sae.at<double>(is[i][1]);
        const double &t2 = local_sae.at<double>(is[i][0]);
        double dta = t0-t1;
        double dtb = t1-t2;
        bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0;
        if(!valid) continue;
        double error = fabs(1 - dtb/dta);
        if(error > tolerance) continue;
        //valid triplet. calulate the velocity.
        double invt = 2.0 /  (dta + dtb);
        wvel.v _= vs[i].v * invt;
        // wvel.v.u += vs[i].u * invt;
        // wvel.v.v += vs[i].v * invt;
        wvel.c++;
        //test rectification of the vector here. in theory slowest vector is most accurate.
    }
    return wvel;
}

}