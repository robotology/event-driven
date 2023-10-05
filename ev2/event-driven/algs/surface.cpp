#include <event-driven/algs/surface.h>
using namespace ev;

cv::Mat surface::getSurface() 
{
    cv::Mat output; surf(actual_region).convertTo(output, CV_8U);
    return output;
}

void surface::init(int width, int height, int kernel_size, double parameter) 
{
    if (kernel_size % 2 == 0)
        kernel_size++;
    this->kernel_size = kernel_size;  //kernel_size should be odd
    this->half_kernel = kernel_size / 2;
    this->parameter = parameter;

    surf = cv::Mat(height+half_kernel*2, width+half_kernel*2, CV_64F, cv::Scalar(0.0));
    actual_region = {half_kernel, half_kernel, width, height};
}

void surface::temporalDecay(double ts, double alpha) {
    surf *= cv::exp(alpha * (time_now - ts));
    time_now = ts;
}

void surface::spatialDecay(int k) 
{
    cv::GaussianBlur(surf, surf, cv::Size(k, k), 0);
}