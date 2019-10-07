//
// Created by miacono on 9/13/19.
//

#ifndef VIPT_H
#define VIPT_H

#include <opencv2/opencv.hpp>

namespace ev {

class vIPT {

private:

    cv::Size size_shared;
    cv::Size size_cam[2];
    cv::Mat cam_matrix[2];
    cv::Mat dist_coeff[2];
    cv::Mat projection[2];
    cv::Mat rotation[2];

    cv::Mat forward_map[2];
    cv::Mat reverse_map[2];

    cv::Size offset;

public:

    struct pixel {
        int u; int v;
    };

    vIPT();

    bool configure(const std::string calibContext, const std::string calibFile,
                   const bool rectify);

    bool cam1ForwardTransform(pixel &p);
    bool cam2ForwardTransform(pixel &p);
    bool cam1ReverseTransform(pixel &p);
    bool cam2ReverseTransform(pixel &p);

    bool cam1ForwardTransform(cv::Mat &m);
    bool cam2ForwardTransform(cv::Mat &m);
    bool cam1ReverseTransform(cv::Mat &m);
    bool cam2ReverseTransform(cv::Mat &m);

    bool transposeCam1ToCam2(pixel &p);
    bool transposeCam2ToCam1(pixel &p);
    bool transposeCam1ToCam2(cv::Mat &m);
    bool transposeCam2ToCam1(cv::Mat &m);


//    void getUndistortedEvent(const resolution &resmod, AE &v) const {
//        cv::Vec2i mapPix;
//        if (v.getChannel() == 0)
//            mapPix = leftMap.at<cv::Vec2i>(v.y, v.x);
//        else
//            mapPix = rightMap.at<cv::Vec2i>(v.y, v.x);

//        //truncate to sensor bounds after mapping?
//        if (truncate && (mapPix[0] < 0 ||
//                         mapPix[0] > resmod.width ||
//                         mapPix[1] < 0 ||
//                         mapPix[1] > resmod.height)) {
//            return;
//        }

//        v.x = mapPix[0];
//        v.y = mapPix[1];
//    };
};

}
#endif //vitp_h