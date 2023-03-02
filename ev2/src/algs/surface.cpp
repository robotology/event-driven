#include <event-driven/algs/surface.h>
using namespace ev;

const cv::Mat &surface::getSurface() 
{
    return surf;
}

void surface::init(int width, int height, int kernel_size, double parameter) 
{
    if (kernel_size % 2 == 0)
        kernel_size++;
    this->kernel_size = kernel_size;  //kernel_size should be odd
    this->half_kernel = kernel_size / 2;
    this->parameter = parameter;

    roi_full = cv::Rect(0, 0, width, height);
    roi_raw = cv::Rect(0, 0, kernel_size, kernel_size);
    surf = cv::Mat(height, width, CV_8U, cv::Scalar(0));
}

bool surface::setRoiAndRegion(int x, int y) 
{
    //set the roi around the event location
    roi_raw.x = x - half_kernel;
    roi_raw.y = y - half_kernel;
    //get only the roi in the limits of the surface
    roi_valid = roi_raw & roi_full;
    //set the region
    region = surf(roi_valid);
    //if roi_raw != roi_valid this was an event next to a border
    return !(roi_raw == roi_valid);
}

void surface::temporalDecay(double ts) {
    surf *= cv::exp(parameter * (time_now - ts));
    time_now = ts;
}

void surface::spatialDecay(int k) 
{
    cv::GaussianBlur(surf, surf, cv::Size(k, k), 0);
}

bool TOS::update(int x, int y, double t, int p) 
{
    (void)t;
    (void)p;
    //parameter default = 2
    static const int threshold = 255 - kernel_size * parameter;

    bool border = setRoiAndRegion(x, y);

    //fix this bug that at borders c may not be correct pixel.
    unsigned char &c = region.at<unsigned char>(half_kernel, half_kernel);
    for (auto xi = 0; xi < region.cols; xi++) {
        for (auto yi = 0; yi < region.rows; yi++) {
            unsigned char &p = region.at<unsigned char>(yi, xi);
            if (p < threshold) p = 0;
            if (p > c) p--;
        }
    }
    c = 255;

    return border;
}

bool SITS::update(int x, int y, double t, int p) 
{
    static const int maximum_value = kernel_size * kernel_size;

    bool border = setRoiAndRegion(x, y);

    unsigned char &c = region.at<unsigned char>(half_kernel, half_kernel);
    for (auto xi = 0; xi < region.cols; xi++) {
        for (auto yi = 0; yi < region.rows; yi++) {
            unsigned char &p = region.at<unsigned char>(yi, xi);
            if (p > c) p--;
        }
    }
    c = maximum_value;

    return border;
}

bool EROS::update(int x, int y, double t, int p) 
{
    //parameter default = 0.3
    static double odecay = pow(parameter, 1.0 / kernel_size);

    bool border = setRoiAndRegion(x, y);

    region *= odecay;
    surf.at<char>(y, x) = 255;

    return border;
}

void PIM::init(int width, int height, int kernel_size, double parameter)
{
    surface::init(width, height, kernel_size, parameter);
    surf = cv::Mat(height, width, CV_32F);
}

bool PIM::update(int x, int y, double t, int p)
{
    if (p)
        surf.at<float>(y, x) -= 1.0f;
    else
        surf.at<float>(y, x) += 1.0f;

    return true;
}