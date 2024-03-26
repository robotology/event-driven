#include <event-driven/algs/flow.h>
#include <vector>

namespace ev {

const int zflowBlock::ap = 2;
const cv::Size zflowBlock::ap_region = {ap*2+1, ap*2+1};

const std::vector< std::vector<cv::Point> > zflowBlock::is = {{{0,0},{1,1}},
                                                                    {{0,2},{1,2}}, 
                                                                    {{0,4},{1,3}}, 
                                                                    {{2,4},{2,3}}, 
                                                                    {{4,4},{3,3}}, 
                                                                    {{4,2},{3,2}}, 
                                                                    {{4,0},{3,1}}, 
                                                                    {{2,0},{2,1}}};

const std::vector<cv::Point2d> zflowBlock::vs ={{1,  1}, 
                                                {1,  0}, 
                                                {1, -1}, 
                                                {0, -1}, 
                                                {-1,-1}, 
                                                {-1, 0}, 
                                                {-1, 1}, 
                                                {0,  1}};

// const int aflow::ap = 2;
// const cv::Size aflow::ap_region = {ap*2+1, ap*2+1};

// const std::vector< std::vector<cv::Point> > aflow::is = {{{0,0},{1,1}},
//                                                                     {{0,2},{1,2}}, 
//                                                                     {{0,4},{1,3}}, 
//                                                                     {{2,4},{2,3}}, 
//                                                                     {{4,4},{3,3}}, 
//                                                                     {{4,2},{3,2}}, 
//                                                                     {{4,0},{3,1}}, 
//                                                                     {{2,0},{2,1}}};

// const std::vector<cv::Point2d> aflow::vs ={{1,  1}, 
//                                                 {1,  0}, 
//                                                 {1, -1}, 
//                                                 {0, -1}, 
//                                                 {-1,-1}, 
//                                                 {-1, 0}, 
//                                                 {-1, 1}, 
//                                                 {0,  1}};

}