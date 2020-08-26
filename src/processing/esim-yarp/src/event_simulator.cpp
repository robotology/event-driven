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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>

#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <event-driven/vPort.h>


template<typename T>
T sampleNormalDistribution(
        bool deterministic = false,
        T mean  = T{0.0},
        T sigma = T{1.0})
{
    static std::mt19937 gen_nondeterministic(std::random_device{}());
    static std::mt19937 gen_deterministic(0);
    auto dist = std::normal_distribution<T>(mean, sigma);
    return deterministic ? dist(gen_deterministic) : dist(gen_nondeterministic);
}

struct Config
{
    double Cp;
    double Cm;
    double sigma_Cp;
    double sigma_Cm;
    unsigned long refractory_period_ns;
    bool use_log_image;
    double log_eps;
};

class EsimModule : public yarp::os::RFModule,
                   public yarp::os::Thread
{
    yarp::os::RpcServer rpcPort;

    bool            closing;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imgPortIn;
    Stamp stamp;

    bool is_initialized_;
    long current_time_;
    cv::Mat_<double> ref_values_;
    cv::Mat_<double> last_img_;
    cv::Mat_<double> last_event_timestamp_;
    cv::Size size_;

    Config config_;

    ev::vWritePort eventPortOut;

    /********************************************************/
    bool quit()
    {
        closing = true;
        return true;
    }

    public:
    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {

        std::string moduleName = rf.check("name", yarp::os::Value("esim-yarp"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")));

        closing = false;

        config_.Cp = 0.05;
        config_.Cm = 0.03;
        config_.sigma_Cp = 0;
        config_.sigma_Cm = 0;
        config_.use_log_image = true;
        config_.log_eps = 0.001;
        config_.refractory_period_ns = 10;
        imgPortIn.open("/" + getName("/image:i"));
        eventPortOut.open("/" + getName("/AE:o"));
        Thread::start();
        current_time_ = 0;
        is_initialized_ = false;
        stamp = Stamp(0, yarp::os::Time::now());

        attach(rpcPort);
        return true;
    }

    /********************************************************/
    bool close()
    {
        imgPortIn.interrupt();
        imgPortIn.close();
        rpcPort.close();
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
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
            yInfoOnce() << "Converting the image to log image with eps = " << config_.log_eps << ".";
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
        while (!Thread::isStopping()){
            yarp::sig::ImageOf<yarp::sig::PixelRgb>* yarpImage = imgPortIn.read();
            cv::Mat cvImage = yarp::cv::toCvMat(*yarpImage);
            cv::cvtColor(cvImage, cvImage, CV_RGB2GRAY);
            cv::Mat_<double> floatImage;
            cvImage.convertTo(floatImage, CV_64FC1, 1 / 255.0);
            stamp.update();
            long time_ns = 1e9 * stamp.getTime();
            std::deque<ev::AddressEvent> events = imageCallback(floatImage, time_ns);


            if(!events.empty())
            {
                eventPortOut.write(events, stamp);
            }
        }
    }
};


int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP server not available!";
        return 1;
    }

    EsimModule module;
    yarp::os::ResourceFinder rf;

    cv::setUseOptimized(true);
    cv::setNumThreads(4);

    rf.setVerbose();
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefaultContext("esim-yarp");

    rf.configure(argc,argv);

    return module.runModule(rf);
}



