/*
 * The EventSimulator takes as input a sequence of stamped images,
 * assumed to be sampled at a "sufficiently high" framerate,
 * and simulates the principle of operation of an idea event camera
 * with a constant contrast threshold C.
 * Pixel-wise intensity values are linearly interpolated in time.
 *
 * The pixel-wise voltages are reset with the values from the first image
 * which is passed to the simulator.
 */

#include <yarp/os/all.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>

#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <event-driven/vPort.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace cv;
using namespace ev;
using std::deque;

template<typename T> T sampleNormalDistribution(bool deterministic = false,
                                                T mean  = T{0.0},
                                                T sigma = T{1.0})
{
    static std::mt19937 gen_nondeterministic(std::random_device{}());
    static std::mt19937 gen_deterministic(0);
    auto dist = std::normal_distribution<T>(mean, sigma);
    return deterministic ? dist(gen_deterministic) : dist(gen_nondeterministic);
}



class EsimModule : public yarp::os::RFModule,
                   public yarp::os::Thread
{

private:

    BufferedPort< ImageOf<PixelRgb> > imgPortIn;
    ev::vWritePort eventPortOut;

    bool is_initialized_;
    long current_time_;
    cv::Mat_<double> ref_values_;
    cv::Mat_<double> last_img_;
    cv::Mat_<double> last_event_timestamp_;
    cv::Size size_;

    struct
    {
        double Cp;
        double Cm;
        double sigma_Cp;
        double sigma_Cm;
        double log_eps;
        unsigned long refractory_period_ns;
        bool use_log_image;
    } config_;

public:

    bool configure(yarp::os::ResourceFinder &rf)
    {

        std::string moduleName = rf.check("name", Value("/esim-yarp"), "module name (string)").asString();
        setName(moduleName.c_str());

        yarp::os::Network yarp;
        if (!yarp.checkNetwork(2.0))
        {
            yError()<<"YARP server not available!";
            return false;
        }

        if(!imgPortIn.open(getName("/image:i"))) {
            yError() << "Could not open" << getName("/image:i");
            return false;
        }

        if(!eventPortOut.open(getName("/AE:o"))) {
            yError() << "Could not open" << getName("/AE:o");
            return false;
        }

        config_.Cp = rf.check("Cp", Value(0.05)).asDouble();
        config_.Cm = rf.check("Cm", Value(0.03)).asDouble();
        config_.sigma_Cp = rf.check("sigma_Cp", Value(0.0)).asDouble();
        config_.sigma_Cm = rf.check("sigma_Cm", Value(0.0)).asDouble();
        config_.log_eps = rf.check("log_eps", Value(0.001)).asDouble();
        config_.refractory_period_ns = rf.check("refractory_period_ns", Value(10)).asInt64();
        config_.use_log_image = rf.check("use_log_image", Value(true)).asBool();

        current_time_ = 0;
        is_initialized_ = false;

        return Thread::start();
    }

    bool close()
    {
        imgPortIn.close();
        eventPortOut.close();
        return true;
    }

    double getPeriod()
    {
        return 1;
    }

    bool updateModule()
    {
        return true;
    }

    void init(const cv::Mat_<double> &img, long time)
    {
        size_ = img.size();
        yInfo() << "Initialized event camera simulator with sensor size: " << size_.width << "x" << size_.height ;
        yInfo() << "and contrast thresholds: C+ = " << config_.Cp << " , C- = " << config_.Cm;
        is_initialized_ = true;
        last_img_ = img.clone();
        ref_values_ = cv::Mat_<double>::zeros(size_);
        last_event_timestamp_ = cv::Mat_<double>::zeros(size_);
        current_time_ = time;
    }

    std::deque<ev::AddressEvent> imageCallback(const cv::Mat_<double>& img, long time) {
        yAssert(time > 0);

        cv::Mat_<double> preprocessed_img;
        if (config_.use_log_image) {
            //yInfoOnce() << "Converting the image to log image with eps = " << config_.log_eps << ".";
            yInfo() << "Converting the image to log image with eps = " << config_.log_eps << ".";
            cv::log(config_.log_eps + img, preprocessed_img);
        }

        if (!is_initialized_) {
            init(preprocessed_img, time);
            return {};
        }

        // For each pixel, check if new events need to be generated since the last image sample
        static constexpr double tolerance = 1e-6;
        std::deque<ev::AddressEvent> events;
        unsigned long delta_t_ns = time - current_time_;

        yAssert(delta_t_ns > 0);
        yAssert(img.size() == size_);
        cv::Mat_<double> diffMat = last_img_ - preprocessed_img;
        cv::Mat_<double> noise(diffMat.rows, diffMat.cols);
        cv::randn(noise, 0, 0.01);
        ref_values_ += diffMat + noise;
        cv::Mat abs = cv::abs(ref_values_);
        cv::threshold(abs, abs, tolerance, 1, CV_THRESH_TOZERO);
        abs.convertTo(abs, CV_8UC1);
        cv::Mat eventList;

        cv::findNonZero(abs, eventList);
        for (int i = 0; i < eventList.total(); i++) {
            ev::AddressEvent v;
            int x = eventList.at<cv::Point>(i).x;
            int y = eventList.at<cv::Point>(i).y;
            v.x = x;
            v.y = y;
            v.stamp = time * 1e-9 * ev::vtsHelper::vtsscaler; // To comply with ATIS timestamp
            v.polarity = (ref_values_(y, x) > 0) ? 1 : 0;
            ref_values_(y, x) = 0;
            events.push_back(v);
            last_event_timestamp_(y, x) = static_cast<double>(time) / 1e9;
        }

        last_img_ = preprocessed_img.clone();

        return events;
    }

    void run(){

        Stamp stamp;
        cv::Mat greyscale;
        cv::Mat floatImage;
        static constexpr double pixelscaler = 1.0 / 255.0;

        while (!Thread::isStopping()) {

            yarp::sig::ImageOf<yarp::sig::PixelRgb>* yarpImage = imgPortIn.read();
            if(!yarpImage)
                return;

            stamp.update();

            cv::Mat cvImage = yarp::cv::toCvMat(*yarpImage);
            cv::cvtColor(cvImage, greyscale, CV_RGB2GRAY);
            greyscale.convertTo(floatImage, CV_64FC1, pixelscaler);

            long time_ns = 1e9 * stamp.getTime();
            deque<AE> events = imageCallback(floatImage, time_ns);

            if(!events.empty())
                eventPortOut.write(events, stamp);
        }
    }
};


int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile( "esim-yarp.ini" );
    rf.setDefaultContext("event-driven");
    rf.configure(argc,argv);

    EsimModule module;
    return module.runModule(rf);
}



