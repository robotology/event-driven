#include <event-driven/algs/surface.h>
using namespace ev;

cv::Mat_<double> surface::getSurface()
{
    return surf(actual_region);
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

cv::Mat_<double> AEDSAE::getSurface()
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

void chainSAE::initialise(cv::Size resolution)
{
    normed_sae = cv::Mat(resolution, CV_64F, 0.0);
    int num_pixels = resolution.area();

    node_list.resize(num_pixels, {NULL, 0.0, 0, NULL, NULL});

    for(int i = 0;i < num_pixels - 1; i++)
    {
        int y = i / resolution.width; int x = i % resolution.width; 
        node_list[i].next = &node_list[i+1];
        node_list[i+1].prev = &node_list[i];
        node_list[i].wp = &normed_sae.at<double>(y, x);
        node_list[i].time = 0.0;
    }
    node_list[num_pixels-1].next = NULL;
    node_list[num_pixels-1].wp = &normed_sae.at<double>();
    node_list[num_pixels-1].time = 0.0;
    node_list[0].prev = NULL;

    head = &node_list[0];
    tail = &node_list[num_pixels-1];

    //precompute the surface shape
    precompweight.resize(num_pixels);
    for(int i = 0; i < num_pixels; i++)
        precompweight[i] = pow(1.0-(double)i/num_pixels, 10);
}

void chainSAE::update(int x, int y, double ts, int p)
{
    int pixel_index = y*normed_sae.cols+x;
    node_* cur_position = &node_list[pixel_index];
    cur_position->time = ts;
    *cur_position->wp = 1;

    if(cur_position == tail) return; //do nothing

    //update the nodes that were linking to cur_position
    if(cur_position == head) {
        // before: NULL <-- A <--> B after: NULL <-- B
        cur_position->next->prev = NULL;
        head = cur_position->next;
    } else {
        // before: A <--> B <--> C after:  A <----> C
        cur_position->next->prev = cur_position->prev;
        cur_position->prev->next = cur_position->next;
    }
    //update the cur_position
    cur_position->next = NULL;
    cur_position->prev = tail;
    //update the tail
    tail->next = &node_list[pixel_index];
    tail = &node_list[pixel_index];
}

cv::Mat_<double> chainSAE::getSurface()
{
    int i = 0;
    for(node_* n = tail; n->prev != NULL; n = n->prev) {
        if(*n->wp < 0) break;
        *n->wp = precompweight[i++];
    }
    return normed_sae;
}

void AAE::initialise(cv::Size resolution, int bs)
{
    this->bs = bs;
    nbs = {resolution.width/bs,resolution.height/bs};
    Ne.resize(nbs.area());

    p2Nej = cv::Mat(resolution, CV_32S);
    for(int y = 0; y < resolution.height; y++) {
        for(int x = 0; x < resolution.width; x++) {
            int Ny = y / bs;
            int Nx = x / bs;
            if(Ny >= nbs.height || Nx >= nbs.width)
                p2Nej.at<int>(y, x) = -1;
            else
                p2Nej.at<int>(y, x) = Ny*nbs.width + Nx;
        }
    }
}

void AAE::update(int x, int y, double ts, int p)
{
    //collect events in each regions queue
    int j = p2Nej.at<int>(y, x);
    if(j < 0) return;

    Ne[j].push_back({x, y, ts});
}

cv::Mat_<double> AAE::getSurface()
{
    cv::Mat A = cv::Mat(p2Nej.size(), CV_64F, 0.0);
    for(size_t j = 0; j < Ne.size(); j++) {
        double b = 0;
        for(size_t v = 1; v < Ne[j].size(); v++)
            b += Ne[j][v].ts - Ne[j][v-1].ts; 
        b /= (Ne[j].size() - 1);
        double a_conv = b > 0 ? 1 / sqrt(b) : 0.0;

        double a = 0;
        for(int v = (int)Ne[j].size()-1; v > 0; v--) {
            double dt = Ne[j][v].ts - Ne[j][v-1].ts;
            a = a / (1 + a*dt) + 1;
            A.at<double>(Ne[j][v].y, Ne[j][v].x) += 0.4;
            if(fabs(a - a_conv) / a_conv < 0.02) {
                break;
            }
        }
        Ne[j].clear();
    }
    return A;
}



