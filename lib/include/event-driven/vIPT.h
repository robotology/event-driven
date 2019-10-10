//
// Created by miacono on 9/13/19.
//

#ifndef VIPT_H
#define VIPT_H

#include <opencv2/opencv.hpp>
#include <yarp/os/Bottle.h>

namespace ev {

class vIPT {

private:

    cv::Size size_shared;
    cv::Size size_cam[2];
    cv::Mat cam_matrix[2];
    cv::Mat dist_coeff[2];
    cv::Mat stereo_rotation, stereo_translation;
    cv::Mat projection[2];
    cv::Mat rotation[2];

    cv::Mat point_forward_map[2];
    cv::Mat point_reverse_map[2];
    cv::Mat mat_reverse_map[2];
    cv::Mat mat_forward_map[2];

    cv::Size offset;

    bool importIntrinsics(int cam, yarp::os::Bottle &parameters);
    bool importStereo(yarp::os::Bottle &parameters);
    bool computeForwardReverseMaps(int cam);


public:

    struct pixel {
        int u; int v;
    };

    vIPT();

    bool configure(const std::string calibContext, const std::string calibFile);
    bool showMapProjections();

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


};

}
#endif //vitp_h
