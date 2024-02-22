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

class zflow
{
private:

    static const int ap;
    static const cv::Size ap_region;
    static const std::vector< std::vector<cv::Point> > is;
    static const std::vector<cv::Point2d> vs;

    std::vector< std::vector<double>* > x_dist;
    std::vector< std::vector<double>* > y_dist;
    cv::Mat flow;
    cv::Mat sae;
    double toc{0.0};

    double tolerance{0.15};
    int block_size;
    cv::Size n_blocks;

public:
    void initialise(const cv::Mat_<double> &sae, int block_size)
    {
        this->sae = sae;
        this->block_size = block_size;
        n_blocks = sae.size() / block_size;
        flow = cv::Mat(n_blocks, CV_32FC2);
    }


    cv::Point2d block_update(double tic, cv::Rect block)
    {
        static std::vector<double> x_dist;
        static std::vector<double> y_dist;
        static const cv::Rect inner = {{ap, ap}, sae.size()-cv::Size(ap*2, ap*2)};

        x_dist.clear();
        y_dist.clear();

        block = block & inner;

        for(auto y = block.y; y < block.y + block.height; y++)
            for(auto x = block.x; x < block.x + block.width; x++)
                point_velocity(sae({{x-ap, y-ap}, ap_region}), x_dist, y_dist);

        std::sort(x_dist.begin(), x_dist.end());
        std::sort(y_dist.begin(), y_dist.end());

        return {x_dist[x_dist.size()/2], y_dist[y_dist.size()/2]};
    }

    void update(double tic)
    {
        for(auto y = 0; y < n_blocks.height; y++) {
            for(auto x = 0; x < n_blocks.width; x++) {
                flow.at<cv::Point2d>(y, x) = block_update(tic, 
                    cv::Rect(x*block_size, y*block_size, block_size, block_size));
            }
        }
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
            bool valid = dta > 0 && dtb > 0 && t1 > 0 && t2 > 0;
            if(!valid) continue;
            double error = fabs(1 - dtb/dta);
            if(error > tolerance) continue;
            //valid triplet. calulate the velocity.
            double invt = 2.0 /  (dta + dtb);
            flow_x.push_back(vs[i].x * invt);
            flow_y.push_back(vs[i].y * invt);
        }
    }

    void visualise(cv::Mat &img)
    {
        //extraxt x and y channels
        cv::Mat xy[2]; //X,Y
        cv::split(flow, xy);

        //calculate angle and magnitude
        cv::Mat magnitude, angle;
        cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

        //translate magnitude to range [0;1]
        double mag_max;
        cv::minMaxLoc(magnitude, 0, &mag_max);
        magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

        //build hsv image
        cv::Mat _hsv[3], hsv;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magnitude;
        cv::merge(_hsv, 3, hsv);

        //convert to BGR
        cv::Mat small;
        cv::cvtColor(hsv, small, cv::COLOR_HSV2BGR);
        cv::resize(small, img, sae.size());

    }

};

}