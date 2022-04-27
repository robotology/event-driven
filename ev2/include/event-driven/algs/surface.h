#pragma once

#include <opencv2/opencv.hpp>

namespace ev {

class surface 
{
protected:
    int kernel_size{0};
    int half_kernel{0};
    cv::Rect roi_full, roi_raw, roi_valid;
    double parameter{0};
    double time_now{0};

    cv::Mat surf;
    cv::Mat region;
    cv::Mat sae;

    bool setRoiAndRegion(int x, int y);

public:
   
    const cv::Mat& getSurface();
    virtual void init(int width, int height, int kernel_size, double parameter = 0.0);
    virtual bool update(int x, int y, double ts, int p) = 0;
    void temporalDecay(double ts);
    void spatialDecay(int k);
};

class EROS : public surface
{
public:
    bool update(int x, int y, double t = 0, int p = 0) override;
};

class TOS : public surface
{
public:
    bool update(int x, int y, double t = 0, int p = 0) override;
};

class SITS : public surface
{
public:
    bool update(int x, int y, double t = 0, int p = 0) override;
};

class PIM : public surface
{
public:
    void init(int width, int height, int kernel_size, double parameter = 0.0) override;
    bool update(int x, int y, double t = 0, int p = 0) override;
};

}
