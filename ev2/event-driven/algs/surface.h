#pragma once

#include <opencv2/opencv.hpp>

namespace ev {

class surface 
{
protected:
    int kernel_size{0};
    int half_kernel{0};
    double parameter{0};
    double time_now{0};

    cv::Rect actual_region;
    cv::Mat surf;

public:
   
    virtual cv::Mat getSurface();
    virtual void init(int width, int height, int kernel_size = 5, double parameter = 0.0);
    virtual inline void update(int x, int y, double ts, int p) = 0;
    void temporalDecay(double ts, double alpha);
    void spatialDecay(int k);
};

class EROS : public surface {
   public:
    inline void update(int x, int y, double t = 0, int p = 0) override 
    {
        static double odecay = pow(parameter, 1.0 / kernel_size);
        surf({x, y, kernel_size, kernel_size}) *= odecay;
        surf.at<double>(y+half_kernel, x+half_kernel) = 255.0;
    }
};

class TOS : public surface 
{
   public:
    inline void update(int x, int y, double t = 0, int p = 0) override {
        (void)t; (void)p;
        // parameter default = 2
        static const double threshold = 255.0 - kernel_size * parameter;
        static cv::Rect roi = {0, 0, kernel_size, kernel_size};

        roi.x = x; roi.y = y;
        static cv::Mat region = surf(roi);

        for (auto xi = 0; xi < region.cols; xi++) {
            for (auto yi = 0; yi < region.rows; yi++) {
                double &p = region.at<double>(yi, xi);
                if (p < threshold) p = 0;
                else p--;
            }
        }
        surf.at<double>(y+half_kernel, x+half_kernel) = 255.0;
    }
};

class SITS : public surface {
   public:
    inline void update(int x, int y, double t = 0, int p = 0) override {
        static const int maximum_value = kernel_size * kernel_size;
        static cv::Rect roi = {0, 0, kernel_size, kernel_size};

        roi.x = x; roi.y = y;
        static cv::Mat region = surf(roi);

        double &c = surf.at<double>(y+half_kernel, x+half_kernel);
        for (auto xi = 0; xi < region.cols; xi++) {
            for (auto yi = 0; yi < region.rows; yi++) {
                double &p = region.at<double>(yi, xi);
                if (p > c) p--;
            }
        }
        c = maximum_value;
    }
};

class PIM : public surface {
   public:
    inline void update(int x, int y, double t = 0, int p = 0) override
    {
        if (p)
            surf.at<double>(y+half_kernel, x+half_kernel) -= 1.0f;
        else
            surf.at<double>(y+half_kernel, x+half_kernel) += 1.0f;
    }
};

class SAE : public surface 
{
   public:
    inline void update(int x, int y, double t = 0, int p = 0) override
    {
        surf.at<double>(y+half_kernel, x+half_kernel) = t;
    }
};

class BIN : public surface
{
   public:
    inline void update(int x, int y, double t = 0, int p = 0) override
    {
        surf.at<double>(y+half_kernel, x+half_kernel) = 255.0;
    }
};

// Set of Centre Active Retinal Fields
class CARF
{
friend class SCARF;
private:
    //parameters
    struct pnt {
        int u:13;
        int v:13;
        int p:3;
        int c:3;
    };
    int N{0};

    //variables
    int i{0};
    std::vector<pnt> points;
    cv::Mat img;

public:

    CARF(int N, cv::Mat img) {
        points.resize(N, {0, 0, 0, 0});
        this->N = N;
        this->img = img; //shallow reference
    }

    inline void add(const CARF::pnt &p)
    {
        if(++i >= N) i = 0;
        if(points[i].c) img.at<float>(points[i].v, points[i].u) -= 0.5;
        if(p.c) img.at<float>(p.v, p.u) += 0.5;
        points[i] = p;
    }
};

class SCARF
{
private:
    //parameters
    cv::Size count{{0, 0}};
    cv::Size dims{{0, 0}};

    //variables
    cv::Mat img;
    std::vector<CARF> rfs;
    std::vector<std::array<int, 4>> cons_map;

public:

    void initialise(cv::Size img_res, int rf_size, double alpha = 1.0)
    {
        initialise(img_res, {img_res.width/rf_size, img_res.height/rf_size}, alpha);
    }

    void initialise(cv::Size img_res, cv::Size rf_res, double alpha = 1.0)
    {
        img = cv::Mat(img_res, CV_32F);
        count = rf_res;
        dims = {img_res.width / rf_res.width, img_res.height / rf_res.height};
        int N = dims.area() * alpha * 0.5;

        cons_map.resize(img_res.area());
        rfs.resize(rf_res.area(), CARF(N, img));
        
        for(int y = 0; y < img_res.height; y++) {
            for(int x = 0; x < img_res.width; x++) {
                int rfx = x / dims.width;
                int rfy = y / dims.height;
                if(rfx >= count.width || rfy >= count.height)
                    continue;
                
                int i = 0;
                auto &conxs = cons_map[y*img_res.width + x];
                conxs[i++] = rfy*rf_res.width+rfx;

                int ky = y%dims.height;
                int kx = x%dims.width;

                bool top{false}, bot{false}, lef{false}, rig{false};
                if(ky < dims.height * 0.5) 
                    {if(rfy > 0) top = true;}
                else 
                    {if(rfy < count.height-1) bot = true;}
                if(kx < dims.width * 0.5) 
                    {if(rfx > 0) lef = true;}
                else 
                    {if(rfx < count.width-1) rig = true;}

                if(top) conxs[i++] = (rfy-1)*rf_res.width+rfx;
                if(bot) conxs[i++] = (rfy+1)*rf_res.width+rfx;
                if(lef) conxs[i++] = rfy*rf_res.width+rfx-1;
                if(rig) conxs[i++] = rfy*rf_res.width+rfx+1;
                if(top && lef) conxs[i++] = (rfy-1)*rf_res.width+rfx-1;
                if(top && rig) conxs[i++] = (rfy-1)*rf_res.width+rfx+1;
                if(bot && lef) conxs[i++] = (rfy+1)*rf_res.width+rfx-1;
                if(bot && rig) conxs[i++] = (rfy+1)*rf_res.width+rfx+1;

                while(i < 4) conxs[i++] = -1;
            }
        }
    }

    inline void update(const int &u, const int &v, const int &p)
    {
        auto &conxs = cons_map[v*img.cols+u];
        rfs[conxs[0]].add({u, v, p, 1});
        for(int j = 1; j < 4; j++) {
            if(conxs[j] < 0) return;
            rfs[conxs[j]].add({u, v, p, 0});
        }
    }

    cv::Mat getSurface()
    {
        return img;
    }

    std::vector<cv::Point> getList(int u, int v)
    {
        std::vector<cv::Point> p;
        for(auto &i : rfs[v*count.width+u].points)
            if(i.c) p.push_back({i.u, i.v});
        return p;
    }

    int getN(void)
    {
        if(rfs.size()) return rfs[0].N;
        else return -1;
    }
};



}
