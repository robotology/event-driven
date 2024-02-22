#include <event-driven/algs/flow.h>
#include <vector>

namespace ev {

const int zflow::ap = 2;
const cv::Size zflow::ap_region = {ap*2+1, ap*2+1};

const std::vector< std::vector<cv::Point> > zflow::is = {{{0,0},{1,1}},
                                                                    {{0,2},{1,2}}, 
                                                                    {{0,4},{1,3}}, 
                                                                    {{2,4},{2,3}}, 
                                                                    {{4,4},{3,3}}, 
                                                                    {{4,2},{3,2}}, 
                                                                    {{4,0},{3,1}}, 
                                                                    {{2,0},{2,1}}};

const std::vector<cv::Point2d> zflow::vs ={{1,  1}, 
                                                {1,  0}, 
                                                {1, -1}, 
                                                {0, -1}, 
                                                {-1,-1}, 
                                                {-1, 0}, 
                                                {-1, 1}, 
                                                {0,  1}};

}