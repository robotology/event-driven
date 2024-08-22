/*
 *   Copyright (C) 2022 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <numeric>
#include <iostream>

namespace ev {

class zrtFlow
{
private:

    struct {} block;
 
    cv::Mat sae;


public:

    cv::Vec2d add(int u, int v);




};

// class zflowBlock
// {
//     friend class zflow;

// private:
//     cv::Vec3b color;
//     cv::Point2i index;
//     cv::Vec2f flow;
//     std::deque<double> x_dist;
//     std::deque<double> y_dist;
//     cv::Mat sae;
//     double tolerance{0.125};
//     double refracotry_period{0.003};
//     double dt{0.05};
//     size_t N{50};

//     static const int ap;
//     static const int d_coordinate;
//     static const cv::Size ap_region;
//     static const std::vector< std::vector<cv::Point> > is;
//     static const std::vector<cv::Point2d> vs;

// public:

//     void initialise(const cv::Mat_<double> &patch, cv::Point2i i)
//     {
//         index = i;
//         flow = {0.0, 0.0};
//         color = {0, 0, 0};
//         sae = patch; // shallow reference
//         // N = std::max(patch.cols, patch.rows);
//     }

//     bool block_update(double toc)
//     {
//         // toc: refractory_period? zc
//         for(auto y = ap; y < sae.rows - ap; y++)
//             for(auto x = ap; x < sae.cols - ap; x++)
//                 if(sae.at<double>(y, x) > toc)
//                     point_velocity(sae({{x-ap, y-ap}, ap_region}), x_dist, y_dist);
        


//         if(x_dist.size() > N) {
//             // flow[0] = std::accumulate(x_dist.begin(), x_dist.end(), 0.0) / x_dist.size();
//             // flow[1] = std::accumulate(y_dist.begin(), y_dist.end(), 0.0) / y_dist.size();

//             std::sort(x_dist.begin(), x_dist.end());
//             std::sort(y_dist.begin(), y_dist.end());
//             flow = {(float)(x_dist[x_dist.size()/2]), (float)(y_dist[y_dist.size()/2])};
//             // while(x_dist.size() > N) {
//             //     x_dist.pop_front(); y_dist.pop_front();
//             // }
            
//             x_dist.clear(); y_dist.clear();
//             //updateColor();
//             return true;
//         }

//         return false;
//     }

//     bool block_update_zc(double toc)
//     {
//         // toc: refractory_period? zc
//         for(auto y = d_coordinate; y < sae.rows - d_coordinate; y++)
//             for(auto x = d_coordinate; x < sae.cols - d_coordinate; x++)
//                 if(sae.at<double>(y, x) > toc)
//                     point_velocity_zc(sae, x_dist, y_dist);
        


//         if(x_dist.size() > N) {
//             // flow[0] = std::accumulate(x_dist.begin(), x_dist.end(), 0.0) / x_dist.size();
//             // flow[1] = std::accumulate(y_dist.begin(), y_dist.end(), 0.0) / y_dist.size();

//             std::sort(x_dist.begin(), x_dist.end());
//             std::sort(y_dist.begin(), y_dist.end());
//             flow = {(float)(x_dist[x_dist.size()/2]), (float)(y_dist[y_dist.size()/2])};
//             // while(x_dist.size() > N) {
//             //     x_dist.pop_front(); y_dist.pop_front();
//             // }
            
//             x_dist.clear(); y_dist.clear();
//             //updateColor();
//             return true;
//         }

//         return false;
//     }

//     void point_velocity(const cv::Mat &local_sae, std::deque<double> &flow_x, std::deque<double> &flow_y)
//     {
//         for(size_t i = 0; i < is.size(); i++) 
//         {
//             const double &t0 = local_sae.at<double>(2, 2);
//             const double &t1 = local_sae.at<double>(is[i][1]);
//             const double &t2 = local_sae.at<double>(is[i][0]);
//             double dta = t0-t1;
//             double dtb = t1-t2;
//             //bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0; //THRESHOLD
//             bool valid = (0 < dta < dt) && (0 < dtb < dt) && t1 > 0 && t2 > 0; //THRESHOLD
//             if(!valid) continue;
//             double error = fabs(1 - dtb/dta);
//             if(error > tolerance/2) continue;          //THRESHOLD
//             //valid triplet. calulate the velocity.
//             double invt = 2.0 /  (dta + dtb);
//             flow_x.push_back(vs[i].x * invt);
//             flow_y.push_back(vs[i].y * invt);
//         }
//     }

//     void point_velocity_zc(const cv::Mat &grid_sae, std::deque<double> &flow_x, std::deque<double> &flow_y)
//     {
//         // for(size_t i = 0; i < is.size(); i++) 
//         // {
//         //     // const double &t0 = local_sae.at<double>(2, 2);
//         //     // const double &t1 = local_sae.at<double>(is[i][1]);
//         //     // const double &t2 = local_sae.at<double>(is[i][0]);
  
//         // }
//         for(size_t i=d_coordinate; i< grid_sae.rows-d_coordinate; i++)
//             for(size_t j=d_coordinate; j<grid_sae.cols-d_coordinate;j++)
//             {
//                 const double &t0 = grid_sae.at<double>(d_coordinate, d_coordinate);
//                 const double &t1 = grid_sae.at<double>(i, j);
//                 const double &t2 = grid_sae.at<double>(2*i-d_coordinate, 2*j-d_coordinate);
//                 double dta = t0-t1;
//                 double dtb = t1-t2;
//                 //bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0; //THRESHOLD
//                 bool valid = (0 < dta < dt) && (0 < dtb < dt) && t1 > 0 && t2 > 0; //THRESHOLD
//                 if(!valid) continue;
//                 double error = fabs(1 - dtb/dta);
//                 if(error > tolerance/2) continue;          //THRESHOLD
//                 //valid triplet. calulate the velocity.
//                 double invt = 2.0 /  (dta + dtb);
//                 flow_x.push_back(vs[i].x * invt);
//                 flow_y.push_back(vs[i].y * invt);
//             }
//     }


// };

// class zflow
// {
// private:

//     std::vector<zflowBlock> blocks;

//     cv::Mat flow;
    
//     cv::Mat sae;
//     double toc{0.0};

//     int block_size;
//     cv::Size n_blocks;

// public:
//     cv::Mat flowbgr;

//     void initialise(const cv::Mat_<double> &sae, int block_size)
//     {
//         this->sae = sae;
//         this->block_size = block_size;
//         n_blocks = sae.size() / block_size;
//         flow = cv::Mat(n_blocks, CV_32FC2);
//         blocks.resize(n_blocks.area());
//         flowbgr = cv::Mat::zeros(sae.size(), CV_8UC3);

//         for(int y = 0; y < n_blocks.height; y++) {
//             for(int x = 0; x < n_blocks.width; x++) {
//                 blocks[y * n_blocks.width + x].initialise(sae({x*block_size, y*block_size, block_size, block_size}), {x, y});
//             }
//         }

//     }

//     void update(double tic)
//     {
//         for(auto &b : blocks) {
//             b.block_update_zc(toc);
//             flow.at<cv::Vec2f>(b.index) = b.flow;
//         }
//         toc = tic;
//     }

//     cv::Mat makebgr()
//     {
//         //extraxt x and y channels
//         cv::Mat xy[2]; //X,Y
//         cv::split(flow, xy);

//         //calculate angle and magnitude
//         cv::Mat magnitude, angle;
//         cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

//         //translate magnitude to range [0;1]
//         // double mag_max;
//         // cv::minMaxLoc(magnitude, 0, &mag_max);
//         // magnitude.convertTo(magnitude, -1, 1.0 / 20.0);
//         cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
//         magnitude *= 0.05;

//         //build hsv image
//         cv::Mat _hsv[3], hsv;
//         _hsv[0] = angle;
//         _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
//         _hsv[2] = magnitude;
//         cv::merge(_hsv, 3, hsv);

//         //convert to BGR
//         cv::Mat small;
//         cv::cvtColor(hsv, small, cv::COLOR_HSV2BGR);
//         small.convertTo(small, CV_8UC3, 255);
//         cv::resize(small, flowbgr, sae.size(), 0.0, 0.0, cv::INTER_NEAREST);
//         return flowbgr;
//     }

    

// };

class zcflowBlock
{
    friend class zcflow;

private:
    cv::Vec3b color;
    cv::Point2i index;
    cv::Vec2f flow;
    // std::deque<double> x_dist;
    // std::deque<double> y_dist;

    std::vector<double> x_dist;
    std::vector<double> y_dist;

    // cv::Mat sae;
    double tolerance{0.125};
    double refracotry_period{0.003};
    double dt{0.05};
    size_t N{30};

    // static const int ap;
    static const int d_coordinate;
    // static const cv::Size ap_region;
    // static const std::vector< std::vector<cv::Point> > is;
    // static const std::vector<cv::Point2d> vs;

public:

    void initialise(cv::Point2i i)
    {
        index = i;
        flow = {0.0,0.0};
        color = {0, 0, 0};
        x_dist.clear();
        y_dist.clear();
        // sae = patch; // shallow reference
        // N = std::max(patch.cols, patch.rows);
    }

    // bool block_update(double toc)
    // {
    //     // toc: refractory_period? zc
    //     for(auto y = ap; y < sae.rows - ap; y++)
    //         for(auto x = ap; x < sae.cols - ap; x++)
    //             if(sae.at<double>(y, x) > toc)
    //                 point_velocity(sae({{x-ap, y-ap}, ap_region}), x_dist, y_dist);
        


    //     if(x_dist.size() > N) {
    //         // flow[0] = std::accumulate(x_dist.begin(), x_dist.end(), 0.0) / x_dist.size();
    //         // flow[1] = std::accumulate(y_dist.begin(), y_dist.end(), 0.0) / y_dist.size();

    //         std::sort(x_dist.begin(), x_dist.end());
    //         std::sort(y_dist.begin(), y_dist.end());
    //         flow = {(float)(x_dist[x_dist.size()/2]), (float)(y_dist[y_dist.size()/2])};
    //         // while(x_dist.size() > N) {
    //         //     x_dist.pop_front(); y_dist.pop_front();
    //         // }
            
    //         x_dist.clear(); y_dist.clear();
    //         //updateColor();
    //         return true;
    //     }

    //     return false;
    // }

    bool block_update_zc(const cv::Mat &sae, int x, int y, cv::Mat &flow_mat, int block_size, cv::Point2i b_index)
    {
        point_velocity_zc(sae, x, y, x_dist, y_dist);
        if(x_dist.size() > N && x_dist.size() < 10000) {
            // flow[0] = std::accumulate(x_dist.begin(), x_dist.end(), 0.0) / x_dist.size();
            // flow[1] = std::accumulate(y_dist.begin(), y_dist.end(), 0.0) / y_dist.size();
            // std::cout << "x_dist_first:" << x_dist.at(1) << std::endl;            
            // std::cout << x_dist.size() << std::endl;
            // std::cout << y_dist.size() << std::endl;

            std::sort(x_dist.begin(), x_dist.end());
            std::sort(y_dist.begin(), y_dist.end());

            float temp_x =0, temp_y = 0;
            cv::Point2i temp;
            int counter_x = 0, counter_y = 0;
            bool flag = 1;
            int rows_range = int(y/block_size)+1;
            int cols_range = int(x/block_size)+1;
            // if(rows_range>0 && cols_range>0 && rows_range<23 &&  cols_range<31)
            for(int j=rows_range-1; j<=rows_range+1; j++) {
                for(int i=cols_range-1; i<=cols_range+1; i++) {
                    temp.x = fabs(flow_mat.at<cv::Vec2f>(j, i)[0]);
                    temp.y = fabs(flow_mat.at<cv::Vec2f>(j, i)[1]);
                    if(j==rows_range&&i==cols_range)
                        flag = 0;
                    if(fabs(flow_mat.at<cv::Vec2f>(j, i)[0])>0.1 && flag)
                    {
                        temp_x += flow_mat.at<cv::Vec2f>(j, i)[0];
                        counter_x += 1;
                    }
                    if(fabs(flow_mat.at<cv::Vec2f>(j, i)[1])>0.1 && flag)
                    {
                        temp_y += flow_mat.at<cv::Vec2f>(j, i)[1];
                        counter_y += 1;
                    }
                    flag = 1;
                }
            }

            temp_x += (float)(x_dist[x_dist.size()/2]);
            temp_y += (float)(y_dist[y_dist.size()/2]);
            counter_x += 1;
            counter_y += 1;
            temp_x /= counter_x;
            temp_y /= counter_y;
            // b_index.x += 1;
            // b_index.y += 1;
            // std::cout<<"temp_x, temp_y:"<<temp_x<<","<<temp_y<<std::endl;
            flow_mat.at<cv::Vec2f>(b_index) = {temp_x, temp_y};

            // flow = {(float)(x_dist[x_dist.size()/2]), (float)(y_dist[y_dist.size()/2])};
            // while(x_dist.size() > N) {
            //     x_dist.pop_front(); y_dist.pop_front();
            // }
            
            x_dist.clear(); y_dist.clear();
            //updateColor();
            return true;
        }

        return false;
    }

    // void point_velocity(const cv::Mat &local_sae, std::deque<double> &flow_x, std::deque<double> &flow_y)
    // {
    //     for(size_t i = 0; i < is.size(); i++) 
    //     {
    //         const double &t0 = local_sae.at<double>(2, 2);
    //         const double &t1 = local_sae.at<double>(is[i][1]);
    //         const double &t2 = local_sae.at<double>(is[i][0]);
    //         double dta = t0-t1;
    //         double dtb = t1-t2;
    //         //bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0; //THRESHOLD
    //         bool valid = (0 < dta < dt) && (0 < dtb < dt) && t1 > 0 && t2 > 0; //THRESHOLD
    //         if(!valid) continue;
    //         double error = fabs(1 - dtb/dta);
    //         if(error > tolerance/2) continue;          //THRESHOLD
    //         //valid triplet. calulate the velocity.
    //         double invt = 2.0 /  (dta + dtb);
    //         flow_x.push_back(vs[i].x * invt);
    //         flow_y.push_back(vs[i].y * invt);
    //     }
    // }

    void point_velocity_zc(const cv::Mat &sae, int x, int y, std::vector<double> &flow_x, std::vector<double> &flow_y)
    {
        // for(size_t i = 0; i < is.size(); i++) 
        // {
        //     // const double &t0 = local_sae.at<double>(2, 2);
        //     // const double &t1 = local_sae.at<double>(is[i][1]);
        //     // const double &t2 = local_sae.at<double>(is[i][0]);
  
        // }
        for(int j=y - d_coordinate; j <= y + d_coordinate; j++)
            for(int i= x - d_coordinate; i<= x + d_coordinate; i++)
            {   
                if(i!=x or j!=y){
                    const double dt12 = sae.at<double>(y, x) - sae.at<double>(j, i);
                    if(0 < dt12 && dt12 < dt)
                    {
                            //
                            const int m = 2 * i - x;
                            const int n = 2 * j - y;
                            
                            // std::cout<<'m'<<m<<std::endl;
                            // std::cout<<'n'<<n<<std::endl;
                            // std::cout<<'i'<<i<<std::endl;
                            // std::cout<<'j'<<j<<std::endl;
                            if(0<=n && n<=sae.rows-1 &&0<=m && m<=sae.cols-1)
                            {    
                                const double dt23 = sae.at<double>(j, i)-sae.at<double>(n, m);
                                double error = fabs(1 - dt23/dt12);
                                if(error > tolerance) continue;          //THRESHOLD
                                //valid triplet. calulate the velocity.
                                double invt = 1.0 /  (dt12 + dt23);
                                // std::cout<<"(x-m) * invt:"<<(x-m) * invt<<std::endl;
                                // std::cout<<"(y-m) * invt:"<<(y-n) * invt<<std::endl;
                                flow_x.push_back(double(x-m) * invt);
                                flow_y.push_back(double(y-n) * invt);
                            }
                    }
                }
                
                // const double &t0 = grid_sae.at<double>(d_coordinate, d_coordinate);
                // const double &t1 = grid_sae.at<double>(i, j);
                // const double &t2 = grid_sae.at<double>(2*i-d_coordinate, 2*j-d_coordinate);
                // double dta = t0-t1;
                // double dtb = t1-t2;
                // //bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0; //THRESHOLD
                // bool valid = (0 < dta < dt) && (0 < dtb < dt) && t1 > 0 && t2 > 0; //THRESHOLD
                // if(!valid) continue;
                // double error = fabs(1 - dtb/dta);
                // if(error > tolerance/2) continue;          //THRESHOLD
                // //valid triplet. calulate the velocity.
                // double invt = 2.0 /  (dta + dtb);
                // flow_x.push_back(vs[i].x * invt);
                // flow_y.push_back(vs[i].y * invt);
            }
    }

};


class zcflow
{
private:

    std::vector<zcflowBlock> blocks;

    cv::Mat flow;
    cv::Mat flow_;
    cv::Mat flow_x;
    cv::Mat flow_y;

    cv::Mat sae_p;
    cv::Mat sae_n;

    double toc{0.0};

    int block_size;
    cv::Size n_blocks;
    cv::Size flow_blocks;
    cv::Point2i b_index;


public:
    cv::Mat flowbgr;
    cv::Mat xy[2]; //X,Y  
    int camera_size_compensation = 1;
    int boundary_compensation = 2;

    void initialise(const cv::Mat_<double> &sae_p, const cv::Mat_<double> &sae_n, int block_size)
    {
        this->sae_p = sae_p;
        this->sae_n = sae_n;
        this->block_size = block_size;
        n_blocks = sae_p.size() / block_size;
        n_blocks.width+=boundary_compensation;
        n_blocks.height+=boundary_compensation;
        n_blocks.width+=camera_size_compensation;
        flow_blocks.height = n_blocks.height;
        flow_blocks.width = n_blocks.width;
        flow = cv::Mat::zeros(flow_blocks, CV_32FC2);
        blocks.resize(n_blocks.area());
        flowbgr = cv::Mat::zeros(sae_p.size(), CV_8UC3);
        flow_x = cv::Mat::zeros(sae_p.size(), CV_32F);
        flow_y = cv::Mat::zeros(sae_p.size(), CV_32F);



        for(int y = 0; y < n_blocks.height; y++) {
            for(int x = 0; x < n_blocks.width; x++) {
                blocks[y * n_blocks.width + x].initialise({x, y});
                // std::cout<<y * n_blocks.width + x<<std::endl;
            }
        }

    }
    

    void update_sae(const cv::Mat_<double> &sae_p, const cv::Mat_<double> &sae_n)
    {
        this->sae_p = sae_p;
        this->sae_n = sae_n;
    }
    
    void clear_blocks()
    {
        for(auto &b : blocks){ 
            flow.at<cv::Vec2f>(b.index) = {0.0, 0.0};
            b.x_dist.clear();
            b.y_dist.clear();
            // std::cout<<""<<b.index<<std::endl;
        }
    }

    void update(double tic)
    {
        // for(auto &b : blocks) {
        //     b.block_update_zc(toc);
        //     flow.at<cv::Vec2f>(b.index) = b.flow;
        // }

        // every time initialize flow
        // for(auto &b : blocks) {
        //     flow.at<cv::Vec2f>(b.index) = {0.0, 0.0};
        //     b.x_dist.clear();
        //     b.y_dist.clear();
        // }

        //for(int y=0; y< sae_p.rows; y++)
            //for(int x=0; x<sae_p.cols; x++)
        // double minVal, maxVal;
        // cv::minMaxLoc(flow, &minVal, &maxVal);
        // std::cout<<"maxVal"<<maxVal<<std::endl;
        // for(int y=0;y<flow.rows;y++)
        //     for(int x=0;x<flow.cols;x++)
        //     {
        //         std::cout<<flow.at<cv::Vec2f>(y, x)[0]<<";"<<flow.at<cv::Vec2f>(y, x)[1]<<std::endl;
        //     }

        for(int y=zcflowBlock::d_coordinate; y< sae_p.rows-zcflowBlock::d_coordinate; y++)
            for(int x=zcflowBlock::d_coordinate; x<sae_p.cols-zcflowBlock::d_coordinate; x++)
            {
                if(sae_p.at<double>(y, x) > toc)
                {
                // sae with block id
                // save triplet connections into x_dist/y_dist to the corresponding block 
                // considering code structure, here we could try to use all triplet connection
                // in each grid in a special time period(the time gap of saving events) 
                // locate the block using (x, y) coordinate
                    b_index = blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].index;
                    b_index.x += 1;
                    b_index.y += 1; 
                    blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].block_update_zc(sae_p, x, y, flow, block_size, b_index);
                }
                if(sae_n.at<double>(y, x) > toc)
                {
                    b_index = blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].index; 
                    b_index.x += 1;
                    b_index.y += 1; 
                    blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].block_update_zc(sae_n, x, y, flow, block_size, b_index);
                }
                // std::cout<<"pixel_counter:"<<pixel_counter<<std::endl;

                    // if connection num> threshold
                    // calculate flow and update it to the buffer
            }
        // // iterate blocks and save flow
        // for(auto &b : blocks) {
        //     // b.block_update_zc(toc);

        //     // flow.at<cv::Vec2f>(b.index) = b.flow;
        //     flow.at<cv::Vec2f>(b.index) = b.flow;

        //     // std::cout<<b.index<<std::endl;
        // }
        toc = tic;
    }

    cv::Mat makebgr()
    {
        //extraxt x and y channels
        // cv::Mat xy[2]; //X,Y

        // flow_ = flow(cv::Range(1, flow.rows-1), cv::Range(1,flow.cols-1));
        // flow_ = flow(cv::Range(0, flow.rows-2), cv::Range(1,flow.cols-2));
        flow_ = flow(cv::Range(1, flow.rows-1), cv::Range(1,flow.cols-1));

  
        cv::split(flow_, xy);

        // cv::resize(xy[0], flow_x, sae_p.size(), 0.0, 0.0, cv::INTER_LINEAR);
        // cv::resize(xy[1], flow_y, sae_p.size(), 0.0, 0.0, cv::INTER_LINEAR);

        // save_results(flow_);

        //calculate angle and magnitude
        cv::Mat magnitude, angle;
        cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

        //translate magnitude to range [0;1]
        // double mag_max;
        // cv::minMaxLoc(magnitude, 0, &mag_max);
        // magnitude.convertTo(magnitude, -1, 1.0 / 20.0);
        cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
        magnitude *= 0.05;

        //build hsv image
        cv::Mat _hsv[3], hsv;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magnitude;
        cv::merge(_hsv, 3, hsv);

        //convert to BGR
        cv::Mat small;
        cv::cvtColor(hsv, small, cv::COLOR_HSV2BGR);
        small.convertTo(small, CV_8UC3, 255);
        cv::resize(small, flowbgr, sae_p.size(), 0.0, 0.0, cv::INTER_LINEAR);
        // cv::bitwise_not(flowbgr, flowbgr);
        return flowbgr;
    }  

    // void save_results(cv::Mat flow_result){
    //     // double minVal; 
    //     // double maxVal; 
    //     // cv::Point minLoc; 
    //     // cv::Point maxLoc;

    //     // minMaxLoc(flow_result, &minVal, &maxVal, &minLoc, &maxLoc );
    //     // std::cout<<'flow_result.max()'<<maxVal<<std::endl;
    //     // std::cout<<'flow_result.min()'<<minVal<<std::endl;

    //     cv::imwrite("/home/lzc/UE_dataset/real_time_flow_results/test.exr", flow_result);

    //     // cv::FileStorage fs("/home/lzc/UE_dataset/real_time_flow_results/test.yml", cv::FileStorage::WRITE);
    //     // fs<<"Image"<<flow_result;

    

    // }



};

// class aflow 
// {
// private:
//     cv::Mat flow, flow2;
    
//     cv::Mat sae;
//     double toc{0.0};

//     int block_size;
//     cv::Size n_blocks;
//     double tolerance{0.1};

//     static const int ap;
//     static const cv::Size ap_region;
//     static const std::vector< std::vector<cv::Point> > is;
//     static const std::vector<cv::Point2d> vs;

// public:
//     cv::Mat flowbgr;

//     void initialise(const cv::Mat_<double> &sae, int block_size)
//     {
//         this->sae = sae;
//         this->block_size = block_size;
//         n_blocks = sae.size() / block_size;
//         flow = cv::Mat(sae.size(), CV_32FC2);
//         flow2 = cv::Mat(sae.size(), CV_32FC2);
//         flowbgr = cv::Mat::zeros(sae.size(), CV_8UC3);

//     }

//     void update(double tic)
//     {
//         for(auto y = ap; y < sae.size().height-ap; y++) {
//             for(auto x = ap; x < sae.size().width-ap; x++) {
//                 if(sae.at<double>(y, x) > toc) {
//                     flow2.at<cv::Vec2f>(y, x) = point_velocity(sae({{x-ap, y-ap}, ap_region}));
//                 }
//             }
//         }
//         //flow = flow2;
//         //cv::medianBlur(flow2, flow, 3);
//         cv::GaussianBlur(flow2, flow, {9, 9}, -1);
//         cv::medianBlur(flow, flow, 3);

//         toc = tic;
//     }

//     cv::Vec2f point_velocity(const cv::Mat &local_sae)
//     {
//         cv::Vec2f f = {0.0, 0.0};
//         int count = 0;
//         for(size_t i = 0; i < is.size(); i++) 
//         {
//             const double &t0 = local_sae.at<double>(2, 2);
//             const double &t1 = local_sae.at<double>(is[i][1]);
//             const double &t2 = local_sae.at<double>(is[i][0]);
//             double dta = t0-t1;
//             double dtb = t1-t2;
//             bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0; //THRESHOLD
//             if(!valid) continue;
//             double error = fabs(1 - dtb/dta);
//             if(error > tolerance) continue;          //THRESHOLD
//             //valid triplet. calulate the velocity.
//             double invt = 2.0 /  (dta + dtb);
//             f[0] += vs[i].x * invt;
//             f[1] += vs[i].y * invt;
//             count++;
//         }
//         if(count) {
//             f[0] /= count;
//             f[1] /= count;
//         }
//         return f;
        
//     }


//     cv::Mat makebgr()
//     {
//         //extraxt x and y channels
//         cv::Mat xy[2]; //X,Y
//         cv::split(flow, xy);

//         //calculate angle and magnitude
//         cv::Mat magnitude, angle;
//         cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

//         //translate magnitude to range [0;1]
//         // double mag_max;
//         // cv::minMaxLoc(magnitude, 0, &mag_max);
//         // magnitude.convertTo(magnitude, -1, 1.0 / 20.0);
//         cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
//         magnitude *= 0.05;

//         //build hsv image
//         cv::Mat _hsv[3], hsv;
//         _hsv[0] = angle;
//         _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
//         _hsv[2] = magnitude;
//         cv::merge(_hsv, 3, hsv);

//         //convert to BGR
//         cv::Mat small;
//         cv::cvtColor(hsv, small, cv::COLOR_HSV2BGR);
//         small.convertTo(small, CV_8UC3, 255);
//         cv::resize(small, flowbgr, sae.size(), 0.0, 0.0, cv::INTER_NEAREST);
//         return flowbgr;
//     }

// };

}
