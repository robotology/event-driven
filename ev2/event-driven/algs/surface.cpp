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

// AEDSAE
void AEDSAE::initialise(cv::Size resolution, double deltat, double lambda)
{
    sae = cv::Mat(resolution, CV_64F);
    if(deltat < 1e-3) deltat = 1e-3; //smallest delta t 1 ms
    this->deltat = deltat;
    this->lambda = lambda;
    filter = cv::getGaussianKernel(resolution.height, resolution.height/2, CV_32F)*
             cv::getGaussianKernel(resolution.width, resolution.width/2, CV_32F).t();
    double maxval;
    cv::minMaxLoc(filter, nullptr, &maxval);
    filter = maxval - filter;
}

void AEDSAE::update(int x, int y, double ts, int p)
{
    sae.at<double>(y, x) = ts;
    event_list.push_back({x, y, ts});
    if(event_list.back().ts < event_list.front().ts) {
        event_list.clear();
        sae.setTo(0.0);
    } else {
        while(event_list.back().ts - event_list.front().ts > deltat)
            event_list.pop_front();
    }

}

//https://github.com/Saleh-I/Filtering-in-frequency-domain/blob/master/Filtering%20in%20frequency%20domain/main.cpp
void fftshift(const cv::Mat &input_img, cv::Mat &output_img)
{
    using cv::Mat, cv::Rect;
	output_img = input_img.clone();
	int cx = output_img.cols / 2;
	int cy = output_img.rows / 2;
	Mat q1(output_img, Rect(0, 0, cx, cy));
	Mat q2(output_img, Rect(cx, 0, cx, cy));
	Mat q3(output_img, Rect(0, cy, cx, cy));
	Mat q4(output_img, Rect(cx, cy, cx, cy));

	Mat temp;
	q1.copyTo(temp);
	q4.copyTo(q1);
	temp.copyTo(q4);
	q2.copyTo(temp);
	q3.copyTo(q2);
	temp.copyTo(q3);
}

void calculateDFT(cv::Mat &scr, cv::Mat &dst)
{
    using cv::Mat;
	// define mat consists of two mat, one for real values and the other for complex values
	Mat planes[] = { scr, Mat::zeros(scr.size(), CV_32F) };
	Mat complexImg;
	cv::merge(planes, 2, complexImg);

	cv::dft(complexImg, complexImg);
	dst = complexImg;
}

void filtering(cv::Mat &scr, cv::Mat &dst, cv::Mat &H)
{
    using cv::Mat, cv::Mat_;
    cv::Mat H2;
	fftshift(H, H2);
	Mat planesH[] = { Mat_<float>(H2.clone()), Mat_<float>(H2) };

	Mat planes_dft[] = { scr, Mat::zeros(scr.size(), CV_32F) };
	split(scr, planes_dft);

	Mat planes_out[] = { Mat::zeros(scr.size(), CV_32F), Mat::zeros(scr.size(), CV_32F) };
	planes_out[0] = planesH[0].mul(planes_dft[0]);
	planes_out[1] = planesH[1].mul(planes_dft[1]);

	merge(planes_out, 2, dst);

}

cv::Mat AEDSAE::getSurface()
{
    
    
    //calculate the binary image
    cv::Mat B = cv::Mat(sae.size(), CV_32F, cv::Scalar(0.0));
    for(auto &v : event_list) {
        B.at<float>(v.y, v.x) = 1.0;
    }

    //keep only high-frequency of B
    cv::Mat dftComplex;
    calculateDFT(B, dftComplex);

    cv::Mat dftComplex_filtered;
    filtering(dftComplex, dftComplex_filtered, filter);

    cv::Mat dft_inverse;
    dft(dftComplex_filtered, dft_inverse, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    cv::threshold(dft_inverse, dft_inverse, 0.001, 1.0, cv::THRESH_BINARY);

    { // - extra visualisation
        // cv::imshow("B", B);
        // cv::Mat bwfilter_show;
        // cv::normalize(filter, bwfilter_show, 1.0, 0.0, cv::NORM_MINMAX);
        // cv::imshow("bwfilter", bwfilter_show);

        // cv::Mat shifted_dftImage;
        // cv::extractChannel(dftComplex, shifted_dftImage, 0);
        // fftshift(shifted_dftImage, shifted_dftImage);
        // cv::normalize(shifted_dftImage, shifted_dftImage, 1.0, 0.0, cv::NORM_MINMAX);
        // shifted_dftImage *= 3;
        // cv::imshow("dftImage", shifted_dftImage);

        // cv::imshow("dftInverse", dft_inverse);
    }

    //count "expected # of events
    double C = cv::countNonZero(dft_inverse); 

    //calculate threshold T to keep approximately "expected # of events" in SAE
    double T = lambda * deltat * C / event_list.size();
    //std::cout << lambda << " " << deltat << " " << C << " " << event_list.size() << " " << T << std::endl; 

    //normlise SAE based on T
    cv::Mat temp = event_list.back().ts - sae;
    temp /= T;

    // calculate 6 order exponent
    cv::Mat temp2;
    temp2 = temp.mul(temp); //^2
    temp = temp.mul(temp2); //^3
    temp = temp.mul(temp);  //^6


    cv::Mat normalised_sae, mask;
    cv::exp(-temp, normalised_sae);
    

    return normalised_sae;
}


