#pragma once

#include <opencv2/opencv.hpp>
#include <tuple>

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
        cv::Mat region = surf(roi);

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
        cv::Mat region = surf(roi);

        double &c = surf.at<double>(y+half_kernel, x+half_kernel);
        for (auto xi = 0; xi < region.cols; xi++) {
            for (auto yi = 0; yi < region.rows; yi++) {
                double &p = region.at<double>(yi, xi);
                if (p >= c) p--;
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
protected:
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
    float C;

public:

    CARF(int N, cv::Mat img, float C = 0.3) {
        points.resize(N, {0, 0, 0, 0});
        this->N = N;
        this->img = img; //shallow reference
        this->C = C;
    }

    inline void add(const CARF::pnt &p)
    {
        if(++i >= N) i = 0;
        if(points[i].c) img.at<float>(points[i].v, points[i].u) -= C;
        if(p.c) img.at<float>(p.v, p.u) += C;
        points[i] = p;
    }
};

class SCARF
{
protected:
    //parameters
    cv::Size count{{0, 0}};
    cv::Size dims{{0, 0}};

    //variables
    cv::Mat img;
    std::vector<CARF> rfs;
    std::vector<std::array<CARF*, 4>> cons_map;

public:

    virtual void initialise(cv::Size img_res, int rf_size, double alpha = 1.0, double C = 0.3)
    {
        if(rf_size % 2) rf_size++;
        initialise(img_res, {(img_res.width/rf_size)-1, (img_res.height/rf_size)-1}, alpha, C);
    }

    virtual void initialise(cv::Size img_res, cv::Size rf_res, double alpha = 1.0, double C = 0.3)
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

    inline virtual void update(const int &u, const int &v, const int &p)
    {
        auto &conxs = cons_map[v*img.cols+u];
        if(conxs[0]) conxs[0]->add({u, v, p, 1});
        if(conxs[1]) conxs[1]->add({u, v, p, 0});
        if(conxs[2]) conxs[2]->add({u, v, p, 0});
        if(conxs[3]) conxs[3]->add({u, v, p, 0});
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

    //THIS COULD BE IMPROVED.
    //INDEXED BY <U, V> - MAYBE A MORE INTUITIVE WAY
    //STORE DATA AS std::vector<cv::Point> would result in this being a straight copy 
    std::vector<cv::Point> getAll(int u, int v)
    {
        std::vector<cv::Point> p;
        for(auto &i : rfs[v*count.width+u].points)
            p.push_back({i.u, i.v});
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

class SCARFflex : public SCARF
{
protected:
    std::vector< std::vector<CARF*> > cons_map;

public:

    void initialise(cv::Size img_res, int rf_size, double alpha = 1.0, double C = 0.3) override
    {
        initialise(img_res, {(img_res.width-(2*10))/rf_size, (img_res.height-(2*10))/rf_size}, alpha, C, 10, 0.5);
    }

    void initialise(cv::Size img_res, cv::Size rf_res, double alpha = 1.0, double C = 0.3) override
    {
        initialise(img_res, rf_res, alpha, C, 10, 0.5);
    }

    void initialise(cv::Size img_res, int rf_size, double alpha = 1.0, double C = 0.3, int b = 10, double r = 0.5)
    {
        initialise(img_res, {(img_res.width-(2*b))/rf_size, (img_res.height-(2*b))/rf_size}, alpha, C, b, r);
    }

    void initialise(cv::Size img_res, cv::Size rf_res, double alpha = 1.0, double C = 0.3, int b = 10, double r = 0.5) 
    {
        img = cv::Mat(img_res, CV_32F);
        count = rf_res;

        //size of a receptive field removing some pixels from the border
        dims = {(img_res.width-(2*b)) / rf_res.width, (img_res.height-(2*b)) / rf_res.height};

        //N is the maximum amount of pixels in the FIFO. Scaled based on active region
        int N = dims.area() * alpha * 0.5;

        //make the CARF receptive fields
        rfs.resize(rf_res.area(), CARF(N, img, C));

        //make the connection map. One entry per pixel
        // pixel -> vector of CARF*
        cons_map.resize(img_res.area()); 

        //for each pixel add in the positive area
        for(int y = 0; y < img_res.height; y++) {
            for(int x = 0; x < img_res.width; x++) {

                //this is the connections from a given pixel
                auto &connection = cons_map[y*img_res.width + x];

                //as we have a border some connections don't have a positive region
                int rfx = std::floor((double)(x-b)/ dims.width);
                int rfy = std::floor((double)(y-b)/ dims.height);
                if(rfx < 0 || rfy < 0 || rfx >= rf_res.width || rfy >= rf_res.height)
                    connection.push_back(nullptr);
                else
                    connection.push_back(&rfs[rfy*rf_res.width+rfx]);
            }
        }

        //for each block add in the negative area
        for(int rfy = 0; rfy < count.height; rfy++) {
            for(int rfx = 0; rfx < count.width; rfx++) {

                CARF* cp = &rfs[rfy*rf_res.width+rfx];

                int yp_start = std::max(b + rfy*dims.height - (int)(dims.height*r), 0);
                int yp_end = std::min(b + (1+rfy)*dims.height + (int)(dims.height*r), img_res.height);
                int xp_start = std::max(b + rfx*dims.width - (int)(dims.width*r), 0);
                int xp_end = std::min(b + (1+rfx)*dims.width + (int)(dims.width*r), img_res.width);
                
                for(int y = yp_start; y < yp_end; y++)
                    for(int x = xp_start; x < xp_end; x++)
                        cons_map[y*img_res.width + x].push_back(cp);
            }
        }
        
    };
    
    inline void update(const int &u, const int &v, const int &p) override
    {
        auto &conxs = cons_map[v*img.cols+u];
        if(conxs[0]) conxs[0]->add({u, v, p, 1});
        for(size_t i = 1; i < conxs.size(); i++)
            if(conxs[i]) conxs[i]->add({u, v, p, 0});
    };

};



}
