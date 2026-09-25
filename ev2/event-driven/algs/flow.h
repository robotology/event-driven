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

// ==================================
// zcflow is Arren's final version
// ==================================

class zrtBlock {
    friend class zrtFlow;
private:
    int N{0};         //this is the maximum number of events to update
    cv::Point2d flow; //raw flow assigned to this block
    std::deque<double> x_dist; //distribution of x connections
    std::deque<double> y_dist; //distribution of y connections

    std::vector<cv::Point> pxs_live, pxs_snap; //live/snap circular buffer
    int i{0}, is{0}; //new event position live/snap
    int j{0}, js{0}; //previously updated position live/snap
    
    double last_update_tic{0}; //use time for flow decay

    //calculate connections for a single event on the SAE
    void singlePixConnections(cv::Mat &sae, int d, double triplet_tolerance, cv::Point p0);

public:

    zrtBlock(int N);

    //add a new point to the block
    void add(cv::Point p);

    //snap saves a copy of the live block into the "snap" variables
    void snap();

    //update connections for each new event
    void updateConnections(cv::Mat &sae, int d, double triplet_tolerance);

    //udate the flow state from the connection buffer
    void updateFlow(size_t n = 0);

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
    double trip_tol{0.125};
    int con_buf_min{20};
    int smooth_factor{3};

public:

    void initialise(cv::Size res, int block_size, int max_N, int connection_length, int con_buf_min, double trip_tol, int smooth_factor);
    
    //add a new event to the SAE and record the new event with the
    //corresponding block
    void add(int u, int v, double t);

    //go through each block and update the list of flow vectors
    //update the final flow per pixel
    void update();

    cv::Mat makebgr();
};

// ==================================
// zcflow is Zhichao's first version
// ==================================

class zcflowBlock
{
    friend class zcflow;

private:
    cv::Vec3b color;
    cv::Point2i index;
    cv::Vec2f flow;

    std::vector<double> x_dist;
    std::vector<double> y_dist;

    // cv::Mat sae;
    double tolerance{0.125};
    //double refracotry_period{0.003};
    double dt{0.05};
    size_t N{30};

    static const int d_coordinate;

public:

    void initialise(cv::Point2i i);

    bool block_update_zc(const cv::Mat &sae, int x, int y, cv::Mat &flow_mat, int block_size, cv::Point2i b_index);

    void point_velocity_zc(const cv::Mat &sae, int x, int y, std::vector<double> &flow_x, std::vector<double> &flow_y);

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

    void initialise(const cv::Mat_<double> &sae_p, const cv::Mat_<double> &sae_n, int block_size);
    
    void update_sae(const cv::Mat_<double> &sae_p, const cv::Mat_<double> &sae_n);
    
    void clear_blocks();

    void update(double tic);

    cv::Mat makebgr();

};

}
