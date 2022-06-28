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
#include <event-driven/core.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "surface.h"

namespace ev 
{

class corner_detector
{
private:

    cv::Mat LUT;
    ev::EROS eros;
    int harris_block_size{7};

    std::thread harris_thread;
    std::mutex m;
    std::condition_variable signal;
    bool eros_updated;
    
    double threshold{0.0};
    double score_mean{0.0};
    double score_variance{0.0};
    int count{0};

    void updateLUT()
    {
        static cv::Mat blurred;
        while(harris_block_size > 0) 
        {
            // std::unique_lock<std::mutex> lk(m);
            // signal.wait(lk, [this]{return eros_updated;});
            // eros_updated = false;
            // lk.unlock();

            cv::GaussianBlur(eros.getSurface(), blurred, cv::Size(5, 5), 0, 0);
            cv::cornerHarris(blurred, LUT, harris_block_size, 3, 0.04);
        }
    }


public:

    ~corner_detector()
    {
        harris_block_size = -1;
        harris_thread.join();
    }

    void initialise(int height, int width, int eros_kernel_size, int harris_block_size)
    {
        if (eros_kernel_size % 2 == 0)
            eros_kernel_size += 1;
        if (harris_block_size % 2 == 0)
            harris_block_size += 1;
        this->harris_block_size = harris_block_size;
        eros.init(width, height, eros_kernel_size, 0.3);
        LUT = cv::Mat(height, width, CV_32F);
        harris_thread = std::thread([this]{updateLUT();});
    }

    template <typename T>
    void detect(T begin, T end, std::deque<AE> &results)
    {
        //first update the EROS
        for(auto &v = begin; v != end; v++) {
            if(eros.update(v->x, v->y)) continue;

            float& score = LUT.at<float>(v->y, v->x);
            if(score > threshold)
                 results.push_back(*v);

            //if (count < 1000000) {
                count++;
                double delta = score - score_mean;
                score_mean += delta / count;
                double delta2 = score - score_mean;
                score_variance += delta * delta2;
           // }
        }
        threshold = score_mean + 2*sqrt(score_variance / count);
        //threshold = 0.00001;

        // std::unique_lock<std::mutex> lk(m);
        // eros_updated = true;
        // lk.unlock();
        // signal.notify_one();


    }

};



}