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
namespace ev {

class zflowBlock
{
    friend class zflow;

private:
    cv::Vec3b color;
    cv::Point2i index;
    cv::Vec2f flow;
    std::vector<double> x_dist;
    std::vector<double> y_dist;
    cv::Mat sae;
    double tolerance{0.1};
    size_t N{1};

    static const int ap;
    static const cv::Size ap_region;
    static const std::vector< std::vector<cv::Point> > is;
    static const std::vector<cv::Point2d> vs;

public:

    void initialise(const cv::Mat_<double> &patch, cv::Point2i i)
    {
        index = i;
        flow = {0.0, 0.0};
        color = {0, 0, 0};
        sae = patch; // shallow reference
        N = std::max(patch.cols, patch.rows);
    }

    bool block_update(double toc)
    {
        for(auto y = ap; y < sae.rows - ap; y++)
            for(auto x = ap; x < sae.cols - ap; x++)
                if(sae.at<double>(y, x) > toc)
                    point_velocity(sae({{x-ap, y-ap}, ap_region}), x_dist, y_dist);

        if(x_dist.size() > N) {
            std::sort(x_dist.begin(), x_dist.end());
            std::sort(y_dist.begin(), y_dist.end());
            flow = {(float)(x_dist[x_dist.size()/2]), (float)(y_dist[y_dist.size()/2])};
            x_dist.clear(); y_dist.clear();
            //updateColor();
            return true;
        }
        return false;
    }

    void point_velocity(const cv::Mat &local_sae, std::vector<double> &flow_x, std::vector<double> &flow_y)
    {
        for(size_t i = 0; i < is.size(); i++) 
        {
            const double &t0 = local_sae.at<double>(2, 2);
            const double &t1 = local_sae.at<double>(is[i][1]);
            const double &t2 = local_sae.at<double>(is[i][0]);
            double dta = t0-t1;
            double dtb = t1-t2;
            bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0; //THRESHOLD
            if(!valid) continue;
            double error = fabs(1 - dtb/dta);
            if(error > tolerance) continue;          //THRESHOLD
            //valid triplet. calulate the velocity.
            double invt = 2.0 /  (dta + dtb);
            flow_x.push_back(vs[i].x * invt);
            flow_y.push_back(vs[i].y * invt);
        }
    }

    void updateColor()
    {
        cv::Mat xm(1, 1, CV_32F); xm.at<float>(0, 0) = flow[0];
        cv::Mat ym(1, 1, CV_32F); ym.at<float>(0, 0) = flow[1];
        //calculate angle and magnitude
        cv::Mat magnitude, angle;
        cv::cartToPolar(xm, ym, magnitude, angle, true);


        // //extraxt x and y channels
        // cv::Mat xy[2]; //X,Y
        // cv::split(flow, xy);

        // //calculate angle and magnitude
        // cv::Mat magnitude, angle;
        // cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

        // magnitude.at<float>(0, 0) *= 0.01;
        // if(magnitude.at<float>(0, 0) > 1.0) magnitude.at<float>(0, 0) = 1.0;

        //build hsv image
        cv::Mat _hsv[3], hsv;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magnitude;
        cv::merge(_hsv, 3, hsv);

        //convert to BGR
        cv::Mat small;
        cv::cvtColor(hsv, small, cv::COLOR_HSV2BGR);
        color = small.at<cv::Vec3b>(0, 0);


        //cartesian to polar
        // double ang = atan2(flow[1], flow[0]) + M_PI;
        // ang *= 255 / (2 * M_PI);

        // double mag = sqrt(flow[1]*flow[1]+flow[0]*flow[0]) * 25.5;
        // if(mag > 255) mag = 255;
        // cv::Mat


        //saturate magnitude

        //hsv to RGB


    }
};

class zflow
{
private:

    std::vector<zflowBlock> blocks;

    cv::Mat flow;
    
    cv::Mat sae;
    double toc{0.0};

    int block_size;
    cv::Size n_blocks;

public:
    cv::Mat flowbgr;

    void initialise(const cv::Mat_<double> &sae, int block_size)
    {
        this->sae = sae;
        this->block_size = block_size;
        n_blocks = sae.size() / block_size;
        flow = cv::Mat(n_blocks, CV_32FC2);
        blocks.resize(n_blocks.area());
        flowbgr = cv::Mat::zeros(sae.size(), CV_8UC3);

        for(int y = 0; y < n_blocks.height; y++) {
            for(int x = 0; x < n_blocks.width; x++) {
                blocks[y * n_blocks.width + x].initialise(sae({x*block_size, y*block_size, block_size, block_size}), {x, y});
            }
        }

    }

    void update(double tic)
    {
        for(auto &b : blocks) {
            if(b.block_update(toc))
                flow.at<cv::Vec2f>(b.index) = b.flow;
            // else
            //     flow.at<cv::Vec2f>(b.index) = {0.0f, 0.0f};
        }
        toc = tic;
    }

    cv::Vec3b colorQuery(int x, int y)
    {
        //get the right block
        x /= block_size; y /= block_size;
        return blocks[y * n_blocks.width + x].color;
    }



    cv::Mat makebgr()
    {
        //extraxt x and y channels
        cv::Mat xy[2]; //X,Y
        cv::split(flow, xy);

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
        cv::resize(small, flowbgr, sae.size(), 0.0, 0.0, cv::INTER_NEAREST);
        return flowbgr;
    }

    

};

}