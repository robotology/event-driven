//
// Created by miacono on 9/13/19.
//

#ifndef ICUB_EVENT_DRIVEN_VUNDISTORT_H
#define ICUB_EVENT_DRIVEN_VUNDISTORT_H

#include <opencv2/opencv.hpp>
#include <yarp/os/ResourceFinder.h>

namespace ev {
    class vUndistort {
    private:
        bool truncate;
        bool rectify;
        cv::Mat leftMap;
        cv::Mat rightMap;

    public:
        vUndistort() = default;

        bool init(
                const bool truncate,
                const bool rectify,
                const int width,
                const int height,
                const std::string calibContext,
                const std::string calibFile) {


            ResourceFinder calibfinder;
            calibfinder.setVerbose();
            calibfinder.setDefaultContext(calibContext);
            calibfinder.setDefaultConfigFile(calibFile);
            calibfinder.configure(0, 0);

            Bottle &leftParams = calibfinder.findGroup("CAMERA_CALIBRATION_LEFT");
            Bottle &rightParams = calibfinder.findGroup("CAMERA_CALIBRATION_RIGHT");
            Bottle &stereoParams = calibfinder.findGroup("STEREO_DISPARITY");
            if (leftParams.isNull() || rightParams.isNull()) {
                yError() << "Could not load intrinsic camera parameters";
                //return false;
            }
            if (rectify && stereoParams.isNull()) {
                yError() << "Could not load extrinsic camera parameters";
                //return false;
            }

            std::cout << leftParams.toString() << std::endl;
            std::cout << rightParams.toString() << std::endl;
            std::cout << stereoParams.toString() << std::endl;


            this->truncate = truncate;
            this->rectify = rectify;
            const yarp::os::Bottle *coeffs[3] = {&leftParams, &rightParams, &stereoParams};
            cv::Mat *maps[2] = {&leftMap, &rightMap};
            cv::Mat cameraMatrix[2];
            cv::Mat distCoeffs[2];
            cv::Mat rectRot[2];
            cv::Size s(height, width);
            cv::Mat Proj[2];

            //create camera and distortion matrices
            for (int i = 0; i < 2; i++) {

                double scaley = height / (double) (coeffs[i]->find("h").asInt());
                double scalex = width / (double) (coeffs[i]->find("w").asInt());

                cameraMatrix[i] = cv::Mat(3, 3, CV_64FC1);
                cameraMatrix[i].setTo(0);
                cameraMatrix[i].at<double>(0, 0) = coeffs[i]->find("fx").asDouble() * scalex;
                cameraMatrix[i].at<double>(1, 1) = coeffs[i]->find("fy").asDouble() * scaley;
                cameraMatrix[i].at<double>(2, 2) = 1.0;
                cameraMatrix[i].at<double>(0, 2) = coeffs[i]->find("cx").asDouble() * scalex;
                cameraMatrix[i].at<double>(1, 2) = coeffs[i]->find("cy").asDouble() * scaley;

                distCoeffs[i] = cv::Mat(4, 1, CV_64FC1);
                distCoeffs[i].at<double>(0, 0) = coeffs[i]->find("k1").asDouble();
                distCoeffs[i].at<double>(0, 1) = coeffs[i]->find("k2").asDouble();
                distCoeffs[i].at<double>(0, 2) = coeffs[i]->find("p1").asDouble();
                distCoeffs[i].at<double>(0, 3) = coeffs[i]->find("p2").asDouble();

                cv::Mat defCamMat = cv::getDefaultNewCameraMatrix(cameraMatrix[i], s, true);
                Proj[i] = defCamMat;
            }

            if (rectify) {
                //Loading extrinsic stereo parameters
                yarp::os::Bottle *HN = coeffs[2]->find("HN").asList();
                if (HN == nullptr || HN->size() != 16)
                    yError() << "Rototranslation matrix HN is absent or without required number of values: 16)";
                else {
                    std::cout << "After extracting list from bottle value HN: " << (HN->toString()) << std::endl;

                    cv::Mat R(3, 3, CV_64FC1); //Rotation matrix between stereo cameras
                    cv::Mat T(3, 1, CV_64FC1); //Translation vector of right wrt left camera center
                    for (int row = 0; row < 3; row++) {
                        for (int col = 0; col < 3; col++) {
                            R.at<double>(row, col) = HN->get(row * 4 + col).asDouble();
                        }
                        T.at<double>(row) = HN->get(row * 4 + 3).asDouble();
                    }
                    std::cout << "R and T values stored properly; R:" << R << "T: " << T << std::endl;

                    cv::Mat R_left(3, 3, CV_64FC1);
                    cv::Mat R_right(3, 3, CV_64FC1);
                    cv::Mat P_left(3, 4, CV_64FC1);
                    cv::Mat P_right(3, 4, CV_64FC1);
                    cv::Mat Q(4, 4, CV_64FC1);
                    //Computing homographies for left and right image
                    cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
                                      s, R, T, R_left, R_right, P_left, P_right, Q, CV_CALIB_ZERO_DISPARITY);
                    rectRot[0] = R_left.clone();
                    rectRot[1] = R_right.clone();
                    Proj[0] = P_left.clone();
                    Proj[1] = P_right.clone();

                }
            }

            for (int i = 0; i < 2; i++) {
                cv::Mat allpoints(height * width, 1, CV_32FC2);
                for (unsigned int y = 0; y < height; y++) {
                    for (unsigned int x = 0; x < width; x++) {
                        allpoints.at<cv::Vec2f>(y * width + x) = cv::Vec2f(x, y);
                    }
                }

                cv::Mat mappoints(height * width, 1, CV_32FC2);

                cv::undistortPoints(allpoints, mappoints, cameraMatrix[i], distCoeffs[i],
                                    rectRot[i], Proj[i]);
                *(maps[i]) = cv::Mat(height, width, CV_32SC2);
                for (unsigned int y = 0; y < height; y++) {
                    for (unsigned int x = 0; x < width; x++) {
                        maps[i]->at<cv::Vec2i>(y, x) =
                                mappoints.at<cv::Vec2f>(y * width + x);
                    }
                }
            }
        }

        void getUndistortedEvent(const resolution &resmod, AE &v) const {
            cv::Vec2i mapPix;
            if (v.getChannel() == 0)
                mapPix = leftMap.at<cv::Vec2i>(v.y, v.x);
            else
                mapPix = rightMap.at<cv::Vec2i>(v.y, v.x);

            //truncate to sensor bounds after mapping?
            if (truncate && (mapPix[0] < 0 ||
                             mapPix[0] > resmod.width ||
                             mapPix[1] < 0 ||
                             mapPix[1] > resmod.height)) {
                return;
            }

            v.x = mapPix[0];
            v.y = mapPix[1];
        };
    };
}
#endif //ICUB_EVENT_DRIVEN_VUNDISTORT_H
