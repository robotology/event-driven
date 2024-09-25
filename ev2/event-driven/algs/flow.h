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
#include <yarp/os/Time.h>

namespace ev {


//so the new plan for zrt flow is to have the sae inside the class.when asking for neighbouring events to create the flow from, it won't be 
//restricted to a block
//each block instead has a list of recent events, a list of recent optical flows,
//the list of recent events limits the number of events that can be processed for real-time operation
//the list of recent optical flows is used to find the mean

class zrtBlock {
    friend class zrtFlow;
private:
    cv::Point2d flow; //raw flow assigned to this block
    std::deque<double> x_dist; //distribution of x connections
    std::deque<double> y_dist; //distribution of y connections
    std::vector<cv::Point> pxs_live; //live circular buffer
    int i{0}, j{0};
    std::vector<cv::Point> pxs_snap; //offline circular buffer
    int is{0}, js{0}; //positions in circular bufer
    int N{0};         //this is the maximum number of events to update
    double last_update_tic{0};

public:

    zrtBlock(int N) {
        this->N = N;
        pxs_live.resize(N);
        flow = {0.0, 0.0};
    }

    //i points to current point in the circular buffer to add new data
    //j points to the last event not yet processed up to the maximum of the circular buffer
    void add(cv::Point p){
        if(++i == N) i = 0;
        pxs_live[i] = p;
        if(i == j) j++;
        if(j == N) j = 0;
    }

    //snap saves a copy of the live block into the "snap" variables
    void snap()
    {
        pxs_snap = pxs_live; //snap the event circular buffer
        is = i; //snap the latest position in circular buffer
        js = j; //snap the oldest position in circular buffer
        j = i;  //set the oldest position to latest position (i.e. all data used)   
    }

    //update connections for each new event
    void updateConnections(cv::Mat &sae, int d, double max_dt, double triplet_tolerance)
    {
        while (js != is) {
            js++;
            if(js == N) js = 0;
            singlePixConnections(sae, d, max_dt, triplet_tolerance, pxs_snap[js]);
        }
    }

    //arren - i think that max_dt and triplet tolerance aren't both necessary and are doing 
    //a similar job. double check

    //calculate connections for a single event on the SAE
    void singlePixConnections(cv::Mat &sae, int d, double max_dt, double triplet_tolerance, cv::Point p0)
    {        
        for(int dy = -d; dy <= d; dy++) {
            for(int dx = -d; dx <= d; dx++) {
                cv::Point p1 = p0 + cv::Point(dx, dy);
                cv::Point p2 = p0 + cv::Point(2*dx, 2*dy);
                if(p2.x < 0 || p2.x >= sae.size().width || p2.y < 0 || p2.y >= sae.size().height)
                    continue;
                double dt12 = sae.at<double>(p0) - sae.at<double>(p1);
                double dt23 = sae.at<double>(p1) - sae.at<double>(p2);
                if(0 < dt12 && dt12 < max_dt && 0 < dt23 && dt23 < max_dt) { //THRESHOLD HERE
                    double error = fabs(1 - dt23/dt12);
                    if(error < triplet_tolerance) { //THRESHOLD HERE
                        double invt = 1.0 /  (dt12 + dt23);
                        x_dist.push_back(dx * invt);
                        y_dist.push_back(dy * invt);
                    }
                }
            }
        }       
    }

    //udate the flow state from the connection buffer
    void updateFlow(size_t n = 0)
    {
        if(n < 3) n = 3;
        if(x_dist.size() < n) {
            double magnitude = sqrt(flow.x*flow.x+flow.y*flow.y);
            double max_mag = 1.0 / (yarp::os::Time::now() - last_update_tic);
            if(magnitude > max_mag) flow *= max_mag / magnitude;
            return;
        } else {
            std::sort(x_dist.begin(), x_dist.end());
            std::sort(y_dist.begin(), y_dist.end());
            flow = {x_dist[x_dist.size()/2], y_dist[y_dist.size()/2]};
            x_dist.clear(); y_dist.clear();
            last_update_tic = yarp::os::Time::now();
        }      
    }

};

class zrtFlow
{
private:

    enum {X=0,Y=1};
 
    cv::Mat sae;
    std::vector<zrtBlock> blocklist;
    std::vector<zrtBlock*> blockmap;
    cv::Size array_dims{{0, 0}};
    cv::Size block_dims{{0, 0}};
    cv::Size image_res{{0, 0}};

    cv::Mat block_flow[2];
    cv::Mat pixel_flow[2];
    cv::Mat full_flow[2];
    cv::Mat hsv, rgb;

    //parameters
    int con_len{3};
    double max_dt{0.05};
    double trip_tol{0.125};
    int con_buf_min{20};
    int smooth_factor{3};

public:

    void initialise(cv::Size res, int block_size, int max_N, int connection_length, int con_buf_min, double max_dt, double trip_tol, int smooth_factor)
    {
        //initialise the SAE
        sae = cv::Mat(res, CV_64F);

        this->con_len = connection_length;
        this->max_dt = max_dt;
        this->trip_tol = trip_tol;
        this->con_buf_min = con_buf_min;
        this->smooth_factor = smooth_factor;

        //calculate blocks
        block_dims = {block_size, block_size};
        array_dims = res / block_size;

        //initialise the blocks
        blocklist.resize(array_dims.area(), zrtBlock(max_N));

        //for speed initialise pointers to blocks for each pixel
        blockmap.resize(res.area());
        for(int y = 0; y < res.height; y++) {
            for(int x = 0; x < res.width; x++) {
                auto &bp = blockmap[y*res.width+x];
                int bx = x / block_dims.width;
                int by = y / block_dims.height;
                if(bx < array_dims.width && by < array_dims.height)
                    bp = &blocklist[by*array_dims.width+bx];
                else
                    bp = nullptr;
            }
        }

        //initialise the flow containers
        //this is the flow 1 pixel per block
        block_flow[X] = cv::Mat::zeros(array_dims, CV_32F);
        block_flow[Y] = cv::Mat::zeros(array_dims, CV_32F);

        //this is the flow at full image size
        full_flow[X] = cv::Mat::zeros(res, CV_32F);
        full_flow[Y] = cv::Mat::zeros(res, CV_32F);

        //this is the flow which might have a 0 border if blocks don't fill the full image space
        pixel_flow[X] = full_flow[X]({0, 0, array_dims.width*block_dims.width, array_dims.height*block_dims.height});
        pixel_flow[Y] = full_flow[Y]({0, 0, array_dims.width*block_dims.width, array_dims.height*block_dims.height});

    }
    
    //add a new event to the SAE and record the new event with the
    //corresponding block
    void add(int u, int v, double t)
    {
        sae.at<double>(v, u) = t;
        blockmap[v*sae.cols+u]->add({u, v});
    }

    //go through each block and update the list of flow vectors
    //update the final flow per pixel
    void update()
    {
        //for each block
        for(int by = 0; by < array_dims.height; by++) {
            for(int bx = 0; bx < array_dims.width; bx++) {
                
                //get the block
                auto &b = blocklist[by * array_dims.width + bx];
                //snapshot the event list so we can process in parallel
                b.snap();

                //calculate the connections for each new pixel and add to blocks flow set
                b.updateConnections(sae, con_len, max_dt, trip_tol);

                //calculate the flow given the connections in
                b.updateFlow(con_buf_min);

                 //asign flow to the array
                block_flow[X].at<float>(by, bx) = b.flow.x;
                block_flow[Y].at<float>(by, bx) = b.flow.y;
            }
        }
        //smooth flow - blockFilter on small image (according to zhichao)
        cv::boxFilter(block_flow[X], block_flow[X], -1, {smooth_factor, smooth_factor});
        cv::boxFilter(block_flow[Y], block_flow[Y], -1, {smooth_factor, smooth_factor});

        // cv::GaussianBlur(block_flow[X], block_flow[X], {smooth_factor, smooth_factor}, -1);
        // cv::GaussianBlur(block_flow[Y], block_flow[Y], {smooth_factor, smooth_factor}, -1);

        //resize flow - with linear interpolation (more smoothing)
        cv::resize(block_flow[X], pixel_flow[X], pixel_flow[X].size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(block_flow[Y], pixel_flow[Y], pixel_flow[Y].size(), 0, 0, cv::INTER_LINEAR);
    }

    cv::Mat makebgr()
    {
        //calculate angle and magnitude
        cv::Mat magnitude, angle;
        cv::cartToPolar(full_flow[X], full_flow[Y], magnitude, angle, true);

        //translate magnitude to range [0;1]
        cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
        magnitude *= 0.05;

        //build hsv image
        cv::Mat _hsv[3];
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magnitude;
        cv::merge(_hsv, 3, hsv);

        //convert to BGR
        cv::Mat rgb32;
        cv::cvtColor(hsv, rgb32, cv::COLOR_HSV2BGR);
        rgb32.convertTo(rgb, CV_8UC3, 255);
        return rgb;
    } 



};

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

}
