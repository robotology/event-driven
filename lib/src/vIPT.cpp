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

#include "event-driven/vIPT.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/all.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace yarp::os;
using std::string;

namespace ev {

vIPT::vIPT()
{
    size_shared = Size(0, 0);
}

bool vIPT::importIntrinsics(int cam, Bottle &parameters)
{
    if(parameters.isNull()) {
        yWarning() << "Could not find camera" << cam <<"parameters";
        return false;
    }

    if(!parameters.check("fx") || !parameters.check("fy") ||
            !parameters.check("cx") || !parameters.check("cy") ||
            !parameters.check("k1") || !parameters.check("k2") ||
            !parameters.check("p1") || !parameters.check("p2")) {
        yError() << "fx,fy,cx,cy,k1,k2,p1, or p2 is missing from"
                 << parameters.toString();
        return false;
    }

    size_cam[cam].height = parameters.find("h").asInt();
    size_cam[cam].width = parameters.find("w").asInt();

    cam_matrix[cam] = cv::Mat(3, 3, CV_64FC1);
    cam_matrix[cam].setTo(0);
    cam_matrix[cam].at<double>(0, 0) = parameters.find("fx").asDouble();
    cam_matrix[cam].at<double>(1, 1) = parameters.find("fy").asDouble();
    cam_matrix[cam].at<double>(2, 2) = 1.0;
    cam_matrix[cam].at<double>(0, 2) = parameters.find("cx").asDouble();
    cam_matrix[cam].at<double>(1, 2) = parameters.find("cy").asDouble();

    dist_coeff[cam] = cv::Mat(4, 1, CV_64FC1);
    dist_coeff[cam].at<double>(0, 0) = parameters.find("k1").asDouble();
    dist_coeff[cam].at<double>(0, 1) = parameters.find("k2").asDouble();
    dist_coeff[cam].at<double>(0, 2) = parameters.find("p1").asDouble();
    dist_coeff[cam].at<double>(0, 3) = parameters.find("p2").asDouble();

    return true;
}

bool vIPT::importStereo(Bottle &parameters)
{

    Bottle *HN = parameters.find("HN").asList();
    if (HN == nullptr || HN->size() != 16) {
        yError() << "Rototranslation matrix HN is absent or without required number of values: 16)";
        return false;
    } else {

        stereo_rotation = cv::Mat(3, 3, CV_64FC1); //Rotation matrix between stereo cameras
        stereo_translation = cv::Mat(3, 1, CV_64FC1); //Translation vector of right wrt left camera center
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                stereo_rotation.at<double>(row, col) = HN->get(row * 4 + col).asDouble();
            }
            stereo_translation.at<double>(row) = HN->get(row * 4 + 3).asDouble();
        }

    }

    return true;
}

bool vIPT::computeForwardReverseMaps(int cam)
{

    point_forward_map[cam] = cv::Mat(size_cam[cam], CV_32SC2);
    mat_forward_map[cam] = cv::Mat(size_cam[cam], CV_32FC2);
    point_reverse_map[cam] = cv::Mat(size_shared, CV_32SC2);
    mat_reverse_map[cam] = cv::Mat(size_shared, CV_32FC2);

    //mat_reverse_map fill
    cv::initUndistortRectifyMap(cam_matrix[cam], dist_coeff[cam], rotation[cam],
                                projection[cam], size_shared, CV_32FC2,
                                mat_reverse_map[cam], cv::noArray());

    if(mat_reverse_map[cam].empty()) {
        yError() << "Camera Calibration failed";
        std::cout << "Camera Matrix: " << std::endl << cam_matrix[cam]
                  << std::endl << std::endl;
        std::cout << "Distortion Coefficients: " << std::endl << dist_coeff[cam]
                  << std::endl << std::endl;
        std::cout << "Rotation Matrix: " << std::endl << rotation[cam]
                  << std::endl << std::endl;
        std::cout << "Projection Matrix: " << std::endl << projection[cam]
                  << std::endl << std::endl;
        std::cout << "Size of Projection Space: " << size_shared << std::endl;
        return false;
    }

    // !!mat_reverse_map is points ordered [x, y]!!

    for(int y = 0; y < size_shared.height; y++) {
        for(int x = 0; x < size_shared.width; x++) {
            cv::Vec2f distorted_point = mat_reverse_map[cam].at<cv::Vec2f>(y, x);

            //point_reverse_map fill
            point_reverse_map[cam].at<cv::Vec2i>(y, x) =
                    cv::Vec2i(distorted_point[1], distorted_point[0]);

            if(distorted_point[1] < 0 ||
                    distorted_point[1] >= size_cam[cam].height ||
                    distorted_point[0] < 0 ||
                    distorted_point[0] >= size_cam[cam].width)
            {
                continue;
            }

            cv::Vec2i dp2 = cv::Vec2i(distorted_point[1], distorted_point[0]);

            //point_forward_map fill
            point_forward_map[cam].at<cv::Vec2i>(dp2) = cv::Vec2i(y, x);

            //mat_forward_map fill (it's used for remap so [x y] format)
            mat_forward_map[cam].at<cv::Vec2f>(dp2) = cv::Vec2f(x, y);
        }
    }

    return true;
}

void vIPT::setProjectedImageSize(int height, int width)
{
    size_shared.height = height;
    size_shared.width = width;
}


const cv::Mat& vIPT::getQ(){
    return Q;
}

bool vIPT::configure(const string calibContext, const string calibFile, int size_scaler)

{
    ResourceFinder calibfinder;
    calibfinder.setVerbose();
    calibfinder.setDefaultContext(calibContext);
    calibfinder.setDefaultConfigFile(calibFile);
    calibfinder.configure(0, 0);

    //import intrinsics
    bool valid_cam1 = importIntrinsics(0, calibfinder.findGroup("CAMERA_CALIBRATION_LEFT"));
    bool valid_cam2 = importIntrinsics(1, calibfinder.findGroup("CAMERA_CALIBRATION_RIGHT"));
    bool valid_stereo = importStereo(calibfinder.findGroup("STEREO_DISPARITY"));

    if(!valid_cam1 && !valid_cam2)
        return false;

    if(size_shared.area() == 0) {
        size_shared = cv::Size2i(cv::max(size_cam[0].width, size_cam[1].width),
                cv::max(size_cam[0].height, size_cam[1].height));
        size_shared *= size_scaler;
    }

    //compute projection and rotation
    if(valid_cam1 && valid_cam2 && valid_stereo) {
        yInfo() << "Camera 0 and Camera 1 is and Extrinsic parameters exist - creating rectified transforms";
        //compute stereo + rectification

        Q = cv::Mat(4, 4, CV_64FC1);

        //Computing homographies for left and right image
        cv::stereoRectify(cam_matrix[0], dist_coeff[0], cam_matrix[1],
                dist_coeff[1], size_cam[0], stereo_rotation, stereo_translation,
                rotation[0], rotation[1], projection[0],projection[1], Q,
                CALIB_ZERO_DISPARITY, 1, size_shared);

    } else {
        if(valid_cam1)
        {

            yInfo() << "Camera 0 is valid - creating optimal camera matrix";

            //compute for camera 1
            projection[0] = cv::getOptimalNewCameraMatrix(cam_matrix[0],
                    dist_coeff[0], size_cam[0], 1, size_shared);
            rotation[0] = cv::Mat::eye(3, 3, CV_32F);
        }
        if(valid_cam2)
        {

            yInfo() << "Camera 1 is valid - creating optimal camera matrix";

            //compute for camera 2
            projection[1] = cv::getOptimalNewCameraMatrix(cam_matrix[1],
                    dist_coeff[1], size_cam[1], 1, size_shared);
            rotation[1] = cv::Mat::eye(3, 3, CV_32F);
        }
    }

    //compute the forward mapping (saving the forward map size and offset)
    if(valid_cam1)
        if(!computeForwardReverseMaps(0))
            return false;
    if(valid_cam2)
        if(!computeForwardReverseMaps(1))
            return false;

    return true;
}

void vIPT::showMonoProjections(int cam, double seconds)
{
    cv::Mat test_image_left(size_cam[cam], CV_8UC1);
    for(int y = 0; y < size_cam[cam].height; y++) {
        for(int x = 0; x < size_cam[cam].width; x++) {
            if(y > size_cam[cam].height / 2) {
                test_image_left.at<uchar>(y, x) = x % 20 > 10 ? 0 : 120;
            } else {
                test_image_left.at<uchar>(y, x) = x % 20 < 10 ? 0 : 120;
            }
        }
    }

    cv::Mat remapped = test_image_left.clone();
    denseForwardTransform(cam, remapped);
    cv::imshow("Original Image", test_image_left);
    cv::imshow("Undistorted Image", remapped);

    cv::waitKey(static_cast<int>(seconds * 1000));
    cv::destroyAllWindows();

}

bool vIPT::showMapProjections(double seconds)
{

    yInfo() << "Showing example projections";

    cv::Mat test_image_left(size_cam[0], CV_8UC1);
    for(int y = 0; y < size_cam[0].height; y++) {
        for(int x = 0; x < size_cam[0].width; x++) {
            if(y > size_cam[0].height / 2) {
                test_image_left.at<uchar>(y, x) = x % 20 > 10 ? 0 : 120;
            } else {
                test_image_left.at<uchar>(y, x) = x % 20 < 10 ? 0 : 120;
            }
        }
    }
    cv::imshow("Cam 0", test_image_left);

    cv::Mat test_image_right(size_cam[1], CV_8UC1);
    for(int y = 0; y < size_cam[1].height; y++) {
        for(int x = 0; x < size_cam[1].width; x++) {
            if(x > size_cam[0].width / 2) {
                test_image_right.at<uchar>(y, x) = y % 20 > 10 ? 0 : 120;
            } else {
                test_image_right.at<uchar>(y, x) = y % 20 < 10 ? 0 : 120;
            }
        }
    }
    cv::imshow("Cam 1", test_image_right);

    yInfo() << "Test image creation success";

    if(this->mat_forward_map[0].empty() || mat_forward_map[1].empty()) {
        yError() << "invalid forward maps";
        return false;
    }
    cv::Mat remapped = test_image_left.clone();
    cv::Mat remapped2 = test_image_right.clone();
    denseForwardTransform(0, remapped);
    denseForwardTransform(1, remapped2);
    remapped = remapped * 0.5 + remapped2 * 0.5;
    cv::imshow("Dense Shared", remapped);

    yInfo() << "Dense Shared success";

    cv::Mat mat_remap_back1 = remapped.clone();
    denseReverseTransform(0, mat_remap_back1);
    cv::imshow("Cam1 Dense Reverse", mat_remap_back1);

    cv::Mat mat_remap_back2= remapped.clone();
    denseReverseTransform(1, mat_remap_back2);
    cv::imshow("Cam2 Dense Reverse", mat_remap_back2);

    yInfo() << "Dense remaps success";

    cv::Mat shared_space = cv::Mat::zeros(size_shared, CV_8UC1);
    for(int y = 0; y < size_cam[0].height; y++) {
        for(int x = 0; x < size_cam[0].width; x++) {
            int xf = x; int yf = y;
            if(sparseForwardTransform(0, yf, xf))
                shared_space.at<uchar>(cv::Vec2i(yf, xf)) += test_image_left.at<uchar>(y, x);
        }
    }

    for(int y = 0; y < size_cam[1].height; y++) {
        for(int x = 0; x < size_cam[1].width; x++) {
            int xf = x; int yf = y;
            if(sparseForwardTransform(1, yf, xf))
                shared_space.at<uchar>(cv::Vec2i(yf, xf)) += test_image_right.at<uchar>(y, x);
        }
    }
    cv::imshow("Sparse Shared", shared_space);
    yInfo() << "Sparse shared success";


    cv::Mat test_left_remapped = cv::Mat::zeros(size_cam[0], CV_8UC1);
    for(int y = 0; y < size_shared.height; y++) {
        for(int x = 0; x < size_shared.width; x++) {
            int xf = x; int yf = y;
            if(sparseReverseTransform(0, yf, xf))
                test_left_remapped.at<uchar>(cv::Vec2i(yf, xf)) = shared_space.at<uchar>(y, x);
        }
    }
    cv::imshow("Cam1 Sparse Reverse", test_left_remapped);

    cv::Mat test_right_remapped = cv::Mat::zeros(size_cam[1], CV_8UC1);
    for(int y = 0; y < size_shared.height; y++) {
        for(int x = 0; x < size_shared.width; x++) {
            int xf = x; int yf = y;
            if(sparseReverseTransform(1, yf, xf))
                test_right_remapped.at<uchar>(cv::Vec2i(yf, xf)) = shared_space.at<uchar>(y, x);
        }
    }
    cv::imshow("Cam2 Sparse Reverse", test_right_remapped);

    yInfo() << "Sparse remaps success";


    cv::waitKey(static_cast<int>(seconds * 1000));
    cv::destroyAllWindows();

    return true;
}

bool vIPT::sparseForwardTransform(int cam, int &y, int &x)
{
    cv::Vec2i p(y, x);
    p = point_forward_map[cam].at<cv::Vec2i>(p);
    y = p[0];
    x = p[1];
    return true;
}

bool vIPT::sparseReverseTransform(int cam, int &y, int &x)
{
    cv::Vec2i p(y, x);
    p = point_reverse_map[cam].at<cv::Vec2i>(p);
    if(p[0] < 0 || p[0] >= size_cam[cam].height ||
            p[1] < 0 || p[1] >= size_cam[cam].width)
    {
        return false;
    }
    y = p[0];
    x = p[1];
    return true;
}


bool vIPT::sparseProjectCam0ToCam1(int &y, int &x)
{
    if(!sparseForwardTransform(0, x, y))
        return false;
    if(!sparseReverseTransform(1, x, y))
        return false;
    return true;
}

bool vIPT::sparseProjectCam1ToCam0(int &y, int &x)
{
    if(!sparseForwardTransform(1, x, y))
        return false;
    if(!sparseReverseTransform(0, x, y))
        return false;
    return true;
}

bool vIPT::denseForwardTransform(int cam, cv::Mat &m)
{
    cv::Mat remapped;
    cv::remap(m, remapped, mat_reverse_map[cam], cv::noArray(), INTER_LINEAR, BORDER_CONSTANT, CV_RGB(255, 255, 255));
    m = remapped;
    return true;
}

bool vIPT::denseReverseTransform(int cam, cv::Mat &m)
{
    cv::Mat remapped;
    cv::remap(m, remapped, mat_forward_map[cam], cv::noArray(), INTER_LINEAR, BORDER_CONSTANT, CV_RGB(255, 255, 255));
    m = remapped;
    return true;
}

bool vIPT::denseProjectCam0ToCam1(cv::Mat &m)
{
    denseForwardTransform(0, m);
    denseReverseTransform(1, m);
    return true;
}
bool vIPT::denseProjectCam1ToCam0(cv::Mat &m)
{
    denseForwardTransform(1, m);
    denseReverseTransform(0, m);
    return true;
}

} //namespace ev::
