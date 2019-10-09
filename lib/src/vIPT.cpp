#include "event-driven/vIPT.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace yarp::os;
using std::string;

namespace ev {

vIPT::vIPT()
{

}

bool vIPT::importIntrinsics(int cam, Bottle &parameters)
{
    if(parameters.isNull())
        return false;

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

}

bool vIPT::importStereo(Bottle &parameters)
{

    Bottle *HN = parameters->find("HN").asList();
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

        yInfo() << "After extracting list from bottle value HN:"
                << (HN->toString());
        yInfo() << "R and T values stored properly; R:"
                << stereo_rotation << "stereo_translation: " << T;
    }

    return true;
}

bool vIPT::computeForwardMap(int cam, cv::Size2i mins, cv::Size2i maxs)
{
    cv::Mat allpoints(size_cam[cam].height * size_cam[cam].width, 1, CV_32FC2);
    for(unsigned int y = 0; y < size_cam[cam].height; y++) {
        for(unsigned int x = 0; x < size_cam[cam].width; x++) {
            allpoints.at<cv::Vec2f>(y * size_cam[cam].width + x) = cv::Vec2f(y, x);
        }
    }

    cv::Mat mappoints;//(res.height * res.width, 1, CV_32FC2);

    cv::undistortPoints(allpoints, mappoints, cam_matrix[cam],
                        dist_coeff[cam], rectRot[cam], Proj[cam],
                        cv::TermCriteria(cv::TermCriteria::COUNT, 1000, 2));

    forward_map[cam] = cv::Mat(size_cam[cam], CV_32SC2);
    for(unsigned int y = 0; y < size_cam[cam].height; y++) {
        for(unsigned int x = 0; x < size_cam[cam].width; x++) {
            cv::Vec2i undistorted_point =
                    mappoints.at<cv::Vec2f>(y * size_cam[cam].width + x) +
                    cv::Vec2f(0.5, 0.5);
            mins.width = std::min(mins.width, undistorted_point[1]);
            mins.height = std::min(mins.height, undistorted_point[0]);
            maxs.width = std::max(maxs.width, undistorted_point[1]);
            maxs.height = std::max(maxs.height, undistorted_point[0]);

            forward_map[cam].at<cv::Vec2i>(y, x) = undistorted_point;
        }
    }
}

bool vIPT::computeReverseMap(int cam)
{

    reverse_map[cam] = cv::Mat(size_shared, CV_32SC2);
    for(unsigned int y = 0; y < size_cam[cam].height; y++) {
        for(unsigned int x = 0; x < size_cam[cam].width; x++) {
            cv::Vec2i undistorted_point = forward_map[i].at<cv::Vec2i>(y, x) + offset;
            reverse_map[cam].at<cv::Vec2i>(undistorted_point) = cv::Vec2i(y, x);
        }
    }

}




bool vIPT::configure(const string calibContext, const string calibFile)

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

    //compute projection and rotation
    if(valid_cam1 && valid_cam2 && valid_stereo) {
        //compute stereo + rectification

        cv::Mat Q(4, 4, CV_64FC1);
        //Computing homographies for left and right image
        cv::stereoRectify(cam_matrix[0], dist_coeff[0],
                cam_matrix[1], dist_coeff[1],
                s, stereo_rotation, stereo_translation,
                rotation[0], rotation[1],
                projection[0],projection[1],
                Q, CV_CALIB_ZERO_DISPARITY);

    } else if(valid_cam1) {
        //compute for camera 1
        projection[0] = cv::getDefaultNewCameraMatrix(cam_matrix[0], s, true);
        rotation[0] = cv::noArray();
    } else if(valid_cam2) {
        //compute for camera 2
        projection[1] = cv::getDefaultNewCameraMatrix(cam_matrix[1], s, true);
        rotation[1] = cv::noArray();
    }

    //compute the forward mapping (saving the forward map size and offset)
    cv::Size2i mins(INT_MAX, INT_MAX);
    cv::Size2i maxs(INT_MAX, INT_MAX);
    if(valid_cam1)
        computeForwardMap(0, mins, maxs);
    if(valid_cam2)
        computeForwardMap(1, mins, maxs);

    offset = -mins;
    size_shared = maxs - mins + cv::Size(1, 1);

    //compute the reverse mapping
    if(valid_cam1)
        computeReverseMap(0);
    if(valid_cam2)
        computeReverseMap(1);

}

bool vIPT::showMapProjections()
{
    cv::Mat test_image_left(size_cam[0], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[0].height; y++) {
        for(unsigned int x = 0; x < size_cam[0].width; x++) {
            if(y > size_cam[0].height / 2) {
                test_image_left.at<uchar>(y, x) = x % 20 > 10 ? 0 : 120;
            } else {
                test_image_left.at<uchar>(y, x) = x % 20 < 10 ? 0 : 120;
            }
        }
    }

    cv::Mat test_image_right(size_cam[1], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[1].height; y++) {
        for(unsigned int x = 0; x < size_cam[1].width; x++) {
            if(x > size_cam[1].width / 2) {
                test_image_right.at<uchar>(y, x) = y % 20 > 10 ? 0 : 120;
            } else {
                test_image_right.at<uchar>(y, x) = y % 20 < 10 ? 0 : 120;
            }
        }
    }

    cv::Mat shared_space = cv::Mat::zeros(size_shared, CV_8UC1);
    for(unsigned int y = 0; y < size_shared.height; y++) {
        for(unsigned int x = 0; x < size_shared.width; x++) {
            cv::Vec2i mapPix = forward_map[0].at<cv::Vec2i>(y, x);
            shared_space.at<uchar>(mapPix + offset) += test_image_left.at<uchar>(y, x);

            mapPix = forward_map[1].at<cv::Vec2i>(y, x);
            shared_space.at<uchar>(mapPix + offset) += test_image_right.at<uchar>(y, x);
        }
    }


    cv::Mat test_left_remapped = cv::Mat::zeros(size_cam[0], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[0].height; y++) {
        for(unsigned int x = 0; x < size_cam[0].width; x++) {
            cv::Vec2i mapPix = forward_map[0].at<cv::Vec2i>(y, x);
            cv::Vec2i redistortedPix = reverse_map[0].at<cv::Vec2i>(mapPix+offset);
            test_left_remapped.at<uchar>(redistortedPix) = test_image_left.at<uchar>(y, x);
        }
    }

    cv::Mat test_right_remapped = cv::Mat::zeros(size_cam[1], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[1].height; y+=1) {
        for(unsigned int x = 0; x < size_cam[1].width; x+=1) {
            cv::Vec2i mapPix = forward_map[1].at<cv::Vec2i>(y, x);
            cv::Vec2i redistortedPix = reverse_map[1].at<cv::Vec2i>(mapPix+offset);
            test_right_remapped.at<uchar>(redistortedPix) = test_image_right.at<uchar>(y, x);
        }
    }

    cv::imshow("Image left", test_image_left);
    cv::imshow("Image right", test_image_right);
    cv::imshow("Image shared space", shared_space);
    cv::imshow("Image unwarped left", test_left_remapped);
    cv::imshow("Image unwarped right", test_right_remapped);
    cv::waitKey(0);

    return true;
}
