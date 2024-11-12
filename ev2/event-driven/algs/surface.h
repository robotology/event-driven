#pragma once

#include <opencv2/opencv.hpp>
#include <tuple>
#include <deque>

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

// Set of Centre Active Receptive Fields
class CARF
{
friend class SCARF;
private:
    //parameters
    struct pnt {
        int p:4;
        int c:4;
    };
    int N{0};

    //variables
    int i{0};
    std::vector<pnt> meta;
    std::vector<cv::Point> all;
    std::vector<cv::Point>::iterator f1, f2;
    std::vector<cv::Point> active;
    cv::Mat img;
    float C;

public:

    CARF(int N, cv::Mat img, float C = 0.3) {
        meta.resize(N, {0, 0});
        all.resize(N, {0, 0});
        active.resize(N+1, {0, 0});
        this->N = N;
        this->img = img; //shallow reference
        this->C = C;
        f1 = active.begin(); f2 = active.begin();
    }

    inline void add(const cv::Point &p, const CARF::pnt &m)
    {
        //if the overwritten point is central
        if(meta[i].c) {
            img.at<float>(all[i]) -= C;
            if(++f1 == active.end()) f1 = active.begin();
        }

        //if the new point is central
        if(m.c) {
            img.at<float>(p) += C;
            *f2 = p;
            if(++f2 == active.end()) f2 = active.begin();
            
        }
        all[i] = p;
        meta[i] = m;
        if(++i >= N) i = 0;
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
    std::vector<std::array<CARF*, 4>> cons_map;

public:

    void initialise(cv::Size img_res, int rf_size, double alpha = 1.0, double C = 0.3)
    {
        if(rf_size % 2) rf_size++;
        initialise(img_res, {(img_res.width/rf_size)-1, (img_res.height/rf_size)-1}, alpha, C);
    }

    void initialise(cv::Size img_res, cv::Size rf_res, double alpha = 1.0, double C = 0.3)
    {
        img = cv::Mat(img_res, CV_32F);
        count = rf_res;

        //size of a receptive field removeing some pixels from the border, make sure the receptive field
        //is an even number
        dims = {img_res.width / (rf_res.width+1), img_res.height / (rf_res.height+1)};
        if(dims.height%2) {dims.height--;} 
        if(dims.width%2) {dims.width--;}

        //N is the maximum amount of pixels in the FIFO
        int N = dims.area() * alpha * 0.5;

        //make the connection map. One entry per pixel . each entry = [id id id id];
        cons_map.resize(img_res.area());
        //make the CARF receptive fields
        rfs.resize(rf_res.area(), CARF(N, img, C));
        
        //for each pixel
        for(int y = 0; y < img_res.height; y++) {
            for(int x = 0; x < img_res.width; x++) {

                auto &connection = cons_map[y*img_res.width + x];
                int i = 0;
                int xm = x - dims.width/2; int ym = y - dims.height/2;

                //as x/y can be negative we allow rfx and rfy to be negative indices.
                int rfx = std::floor((double)xm/ dims.width);
                int rfy = std::floor((double)ym/ dims.height);
                if(rfx < 0 || rfy < 0 || rfx >= rf_res.width || rfy >= rf_res.height)
                    connection[i++] = nullptr;
                else
                    connection[i++] = &rfs[rfy*rf_res.width+rfx];
                
                //fancy modulus to keep ky and kx positive values.
                int ky = (dims.height+(ym%dims.height))%dims.height;
                int kx = (dims.width+(xm%dims.width))%dims.width;

                //first find potential suppression indicies
                bool top{false}, bot{false}, lef{false}, rig{false};
                if(ky < dims.height/2) top = true;
                else bot = true;
                if(kx < dims.width/2) lef = true;
                else rig = true;

                //get all the rf coordinates
                std::vector<cv::Point> potentials;
                if(top) potentials.push_back({rfx, rfy-1});
                if(bot) potentials.push_back({rfx, rfy+1});
                if(lef) potentials.push_back({rfx-1, rfy});
                if(rig) potentials.push_back({rfx+1, rfy});
                if(top && lef) potentials.push_back({rfx-1, rfy-1});
                if(top && rig) potentials.push_back({rfx+1, rfy-1});
                if(bot && lef) potentials.push_back({rfx-1, rfy+1});
                if(bot && rig) potentials.push_back({rfx+1, rfy+1});

                //if the coordinate is valid add a connection;
                for(auto &j : potentials) {
                    if(j.x >= 0 && j.x < rf_res.width && j.y >= 0 && j.y < rf_res.height)
                        connection[i++] = &rfs[j.y*rf_res.width+j.x];
                    else
                        connection[i++] = nullptr;
                }

            }
        }
    }

    inline void update(const int &u, const int &v, const int &p)
    {
        auto &conxs = cons_map[v*img.cols+u];
        if(conxs[0]) conxs[0]->add({u, v}, {p, 1});
        if(conxs[1]) conxs[1]->add({u, v}, {p, 0});
        if(conxs[2]) conxs[2]->add({u, v}, {p, 0});
        if(conxs[3]) conxs[3]->add({u, v}, {p, 0});
    }

    cv::Mat getSurface()
    {
        return img;
    }

    std::vector<cv::Point> getActive(int rf_ix, int rf_iy)
    {
        auto &rf = rfs[rf_iy*count.width+rf_ix];
        if(rf.f1 > rf.f2) {
            std::vector<cv::Point> temp = {rf.f1, rf.active.end()};
            temp.insert(temp.end(), rf.active.begin(), rf.f2);
            return temp;
        } else {
            return {rf.f1, rf.f2};
        }
    }

    std::vector<cv::Point> getAll(int rf_ix, int rf_iy)
    {
        return rfs[rf_iy*count.width+rf_ix].all;
    }

    std::vector<cv::Point> getListOld(int u, int v)
    {

        std::vector<cv::Point> p;
        auto &rf = rfs[v*count.width+u];
        for(size_t i = 0; i < rf.all.size(); i++)
            if(rf.meta[i].c) p.push_back(rf.all[i]);
        return p;
    }

    //THIS COULD BE IMPROVED.
    //INDEXED BY <U, V> - MAYBE A MORE INTUITIVE WAY
    //STORE DATA AS std::vector<cv::Point> would result in this being a straight copy 
    std::vector<cv::Point> getAllOld(int u, int v)
    {
        std::vector<cv::Point> p;
        auto &rf = rfs[v*count.width+u];
        for(size_t i = 0; i < rf.all.size(); i++)
            p.push_back(rf.all[i]);
        return p;
    }

    int getN(void)
    {
        if(rfs.size()) return rfs[0].N;
        else return -1;
    }

    /**
     * @brief Get the Scarf Params object
     * 
     * @return std::tuple<cv::Size count, cv::Size dims, int N, cv::Size border_shift>
     *         count        : Resolution of SCARF blocks
     *         dims         : Size of receptive field
     *         N            : Size of ring buffer in each SCARF block
     *         border_shift : (HARD CORDED) pixel values for shift size to match origin of image and origin of scarf
     */
    std::tuple<cv::Size, cv::Size, int, cv::Size> getScarfParams(void)
    {
        int N = -1;
        if (rfs.size()) N = rfs[0].N;
        std::tuple<cv::Size, cv::Size, int, cv::Size> scarf_params = std::make_tuple(
            count,
            dims,
            N,
            cv::Size(dims.width/2, dims.height/2)
        );
        return scarf_params;
    }
};



}
