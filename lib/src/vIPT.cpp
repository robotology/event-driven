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

        yInfo() << "After extracting list from bottle value HN:"
                << (HN->toString());
        //yInfo() << "R and T values stored properly; R:"
        //        << stereo_rotation << "stereo_translation: " << stereo_translation;
    }

    return true;
}

bool vIPT::computeForwardReverseMaps(int cam)
{

    point_forward_map[cam] = cv::Mat(size_cam[cam], CV_32SC2);
    mat_forward_map[cam] = cv::Mat(size_cam[cam], CV_32FC2);

    //mat_reverse_map fill
    cv::initUndistortRectifyMap(cam_matrix[cam], dist_coeff[cam], rotation[cam],
                                projection[cam], size_shared, CV_32FC2,
                                mat_reverse_map[cam], cv::noArray());
    point_reverse_map[cam] = cv::Mat(size_shared, CV_32SC2);
    // !!urmap is points ordered [x, y]!!

    for(unsigned int y = 0; y < size_shared.height; y++) {
        for(unsigned int x = 0; x < size_shared.width; x++) {
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

    size_shared = cv::Size2i(cv::max(size_cam[0].width, size_cam[1].width),
            cv::max(size_cam[1].height, size_cam[1].height))*2;

    //compute projection and rotation
    if(valid_cam1 && valid_cam2 && valid_stereo) {
        //compute stereo + rectification

        cv::Mat Q(4, 4, CV_64FC1);
        //Computing homographies for left and right image
        cv::stereoRectify(cam_matrix[0], dist_coeff[0], cam_matrix[1],
                dist_coeff[1], size_cam[0], stereo_rotation, stereo_translation,
                rotation[0], rotation[1], projection[0],projection[1], Q,
                CV_CALIB_ZERO_DISPARITY, 1, size_shared);

    } else if(valid_cam1) {
        //compute for camera 1
        projection[0] = cv::getOptimalNewCameraMatrix(cam_matrix[0],
                dist_coeff[0], size_cam[0], 1, size_shared);
        rotation[0] = cv::Mat::eye(3, 3, CV_32F);
    } else if(valid_cam2) {
        //compute for camera 2
        projection[1] = cv::getOptimalNewCameraMatrix(cam_matrix[1],
                dist_coeff[1], size_cam[1], 1, size_shared);
        rotation[1] = cv::Mat::eye(3, 3, CV_32F);
    }

    //compute the forward mapping (saving the forward map size and offset)
    if(valid_cam1)
        computeForwardReverseMaps(0);
    if(valid_cam2)
        computeForwardReverseMaps(1);

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
    cv::imshow("Cam 0", test_image_left);

    cv::Mat test_image_right(size_cam[1], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[1].height; y++) {
        for(unsigned int x = 0; x < size_cam[1].width; x++) {
            if(x > size_cam[0].width / 2) {
                test_image_right.at<uchar>(y, x) = y % 20 > 10 ? 0 : 120;
            } else {
                test_image_right.at<uchar>(y, x) = y % 20 < 10 ? 0 : 120;
            }
        }
    }
    cv::imshow("Cam 1", test_image_right);

    cv::Mat remapped, remapped2;
    cv::remap(test_image_left, remapped, mat_reverse_map[0], cv::noArray(),
            CV_INTER_LINEAR);
    cv::remap(test_image_right, remapped2, mat_reverse_map[1], cv::noArray(),
            CV_INTER_LINEAR);
    remapped = remapped * 0.5 + remapped2 * 0.5;
    cv::imshow("Dense Shared", remapped);

    cv::Mat mat_remap_back1;
    cv::remap(remapped, mat_remap_back1, mat_forward_map[0], cv::noArray(), CV_INTER_LINEAR);
    cv::imshow("Cam1 Dense Reverse", mat_remap_back1);

    cv::Mat mat_remap_back2;
    cv::remap(remapped, mat_remap_back2, mat_forward_map[1], cv::noArray(), CV_INTER_LINEAR);
    cv::imshow("Cam2 Dense Reverse", mat_remap_back2);

    cv::Mat shared_space = cv::Mat::zeros(size_shared, CV_8UC1);
    for(unsigned int y = 0; y < size_cam[0].height; y++) {
        for(unsigned int x = 0; x < size_cam[0].width; x++) {
            cv::Vec2i mapPix = point_forward_map[0].at<cv::Vec2i>(y, x);
            shared_space.at<uchar>(mapPix) += test_image_left.at<uchar>(y, x);
        }
    }
    for(unsigned int y = 0; y < size_cam[1].height; y++) {
        for(unsigned int x = 0; x < size_cam[1].width; x++) {
            cv::Vec2i mapPix = point_forward_map[1].at<cv::Vec2i>(y, x);
            shared_space.at<uchar>(mapPix) += test_image_right.at<uchar>(y, x);
        }
    }
    cv::imshow("Sparse Shared", shared_space);


    cv::Mat test_left_remapped = cv::Mat::zeros(size_cam[0], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[0].height; y++) {
        for(unsigned int x = 0; x < size_cam[0].width; x++) {
            cv::Vec2i mapPix = point_forward_map[0].at<cv::Vec2i>(y, x);
            cv::Vec2i redistortedPix = point_reverse_map[0].at<cv::Vec2i>(mapPix);
            if(redistortedPix[0] < 0 || redistortedPix[0] >= size_cam[0].height ||
                    redistortedPix[1] < 0 || redistortedPix[1] >= size_cam[0].width) {
                std::cout << redistortedPix << std::endl;
                continue;
            }
            test_left_remapped.at<uchar>(redistortedPix) = test_image_left.at<uchar>(y, x);
        }
    }
    cv::imshow("Image unwarped cam1", test_left_remapped);

    cv::Mat test_right_remapped = cv::Mat::zeros(size_cam[1], CV_8UC1);
    for(unsigned int y = 0; y < size_cam[1].height; y++) {
        for(unsigned int x = 0; x < size_cam[1].width; x++) {
            cv::Vec2i mapPix = point_forward_map[1].at<cv::Vec2i>(y, x);
            cv::Vec2i redistortedPix = point_reverse_map[1].at<cv::Vec2i>(mapPix);
            if(redistortedPix[0] < 0 || redistortedPix[0] >= size_cam[1].height ||
                    redistortedPix[1] < 0 || redistortedPix[1] >= size_cam[1].width) {
                std::cout << redistortedPix << std::endl;
                continue;
            }
            test_right_remapped.at<uchar>(redistortedPix) = test_image_right.at<uchar>(y, x);

        }
    }
    cv::imshow("Image unwarped cam2", test_right_remapped);

    cv::waitKey(0);

    return true;
}

} //namespace ev::
