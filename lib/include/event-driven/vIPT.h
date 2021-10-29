/*
 *   Copyright (C) 2019 Event-driven Perception for Robotics
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
#include <yarp/os/all.h>

namespace ev {

//event-based inverse pixel transform
class vIPT {

private:

    cv::Size size_shared;
    cv::Size size_cam[2];
    cv::Mat cam_matrix[2];
    cv::Mat dist_coeff[2];
    cv::Mat stereo_rotation, stereo_translation;
    cv::Mat projection[2];
    cv::Mat rotation[2];
    cv::Mat Q;

    cv::Mat point_forward_map[2];
    cv::Mat point_reverse_map[2];
    cv::Mat mat_reverse_map[2];
    cv::Mat mat_forward_map[2];

    bool importIntrinsics(int cam, yarp::os::Bottle &parameters);
    bool importStereo(yarp::os::Bottle &parameters);
    bool computeForwardReverseMaps(int cam);


public:

    vIPT();

    const cv::Mat& getQ();
    void setProjectedImageSize(int height, int width);
    bool configure(const std::string &calib_file_path, int size_scaler = 2);
    bool showMapProjections(double seconds = 0);
    void showMonoProjections(int cam, double seconds);
    void printValidCalibrationValues();

    bool sparseForwardTransform(int cam, int &y, int &x);
    bool sparseReverseTransform(int cam, int &y, int &x);
    bool sparseProjectCam0ToCam1(int &y, int &x);
    bool sparseProjectCam1ToCam0(int &y, int &x);

    bool denseForwardTransform(int cam, cv::Mat &m);
    bool denseReverseTransform(int cam, cv::Mat &m);
    bool denseProjectCam0ToCam1(cv::Mat &m);
    bool denseProjectCam1ToCam0(cv::Mat &m);


};

// def draw_axis(img, R, t, K):
//     # unit is mm
//     rotV, _ = cv2.Rodrigues(R)
//     points = np.float32([[100, 0, 0], [0, 100, 0], [0, 0, 100], [0, 0, 0]]).reshape(-1, 3)
//     axisPoints, _ = cv2.projectPoints(points, rotV, t, K, (0, 0, 0, 0))
//     img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
//     img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
//     img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
//     return img
cv::Mat drawRefAxis();
cv::Mat drawRefAxis(std::array<double, 4> q);
cv::Mat drawRefAxis(std::array<float, 3> r);
cv::Mat drawRefAxis(std::array<float, 3> r, cv::Mat K);


}

