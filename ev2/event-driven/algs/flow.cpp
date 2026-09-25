#include <event-driven/algs/flow.h>
#include <vector>

namespace ev {

// ==================================
// zrtFlow is Arren's final version, more real-time emphasis
// ==================================

zrtBlock::zrtBlock(int N) {
    this->N = N;
    pxs_live.resize(N);
    flow = {0.0, 0.0};
}

//i points to current point in the circular buffer to add new data
//j points to the last event not yet processed up to the maximum of the circular buffer
void zrtBlock::add(cv::Point p){
    if(++i == N) i = 0;
    pxs_live[i] = p;
    if(i == j) j++;
    if(j == N) j = 0;
}

//snap saves a copy of the live block into the "snap" variables
void zrtBlock::snap()
{
    pxs_snap = pxs_live; //snap the event circular buffer
    is = i; //snap the latest position in circular buffer
    js = j; //snap the oldest position in circular buffer
    j = i;  //set the oldest position to latest position (i.e. all data used) 

    if(is == N) is = 0; //thread "safety" if snapped during i++ command in "add"
    if(js == N) js = 0; //thread "safety" if snapped during j++ command in "add"
}

//calculate connections for a single event on the SAE
void zrtBlock::singlePixConnections(cv::Mat &sae, int d, double triplet_tolerance, cv::Point p0)
{        
    for(int dy = -d; dy <= d; dy++) {
        for(int dx = -d; dx <= d; dx++) {
            cv::Point p1 = p0 + cv::Point(dx, dy);
            cv::Point p2 = p0 + cv::Point(2*dx, 2*dy);
            if(p2.x < 0 || p2.x >= sae.size().width || p2.y < 0 || p2.y >= sae.size().height)
                continue;
            double dt12 = sae.at<double>(p0) - sae.at<double>(p1);
            double dt23 = sae.at<double>(p1) - sae.at<double>(p2);
            if(0 < dt12 && 0 < dt23) {
                double error = fabs(1 - dt23/dt12);
                if(error < triplet_tolerance) { //THRESHOLD HERE
                    double invt = 1.0 /  (dt12 + dt23);
                    x_dist.push_back(dx * invt);
                    y_dist.push_back(dy * invt);
                }
            }
        }
    }       
}

//update connections for each new event
void zrtBlock::updateConnections(cv::Mat &sae, int d, double triplet_tolerance)
{
    while (js != is) {
        js++;
        if(js == N) js = 0;
        singlePixConnections(sae, d, triplet_tolerance, pxs_snap[js]);
    }
}

//udate the flow state from the connection buffer
void zrtBlock::updateFlow(size_t n)
{
    if(n < 3) n = 3;
    if(x_dist.size() < n) {
        double magnitude = sqrt(flow.x*flow.x+flow.y*flow.y);
        double max_mag = 1.0 / (yarp::os::Time::now() - last_update_tic);
        if(magnitude > max_mag) flow *= max_mag / magnitude;
        return;
    } else {
        std::sort(x_dist.begin(), x_dist.end());
        std::sort(y_dist.begin(), y_dist.end());
        flow = {x_dist[x_dist.size()/2], y_dist[y_dist.size()/2]};
        x_dist.clear(); y_dist.clear();
        last_update_tic = yarp::os::Time::now();
    }      
}

void zrtFlow::initialise(cv::Size res, int block_size, int max_N, int connection_length, int con_buf_min, double trip_tol, int smooth_factor)
{
    //initialise the SAE
    sae = cv::Mat(res, CV_64F);

    this->con_len = connection_length;
    this->trip_tol = trip_tol;
    this->con_buf_min = con_buf_min;
    this->smooth_factor = smooth_factor;

    //calculate blocks
    block_dims = {block_size, block_size};
    array_dims = res / block_size;

    //initialise the blocks
    blocklist.resize(array_dims.area(), zrtBlock(max_N));

    //for speed initialise pointers to blocks for each pixel
    blockmap.resize(res.area());
    for(int y = 0; y < res.height; y++) {
        for(int x = 0; x < res.width; x++) {
            auto &bp = blockmap[y*res.width+x];
            int bx = x / block_dims.width;
            int by = y / block_dims.height;
            if(bx < array_dims.width && by < array_dims.height)
                bp = &blocklist[by*array_dims.width+bx];
            else
                bp = nullptr;
        }
    }

    //initialise the flow containers
    //this is the flow 1 pixel per block
    block_flow[X] = cv::Mat::zeros(array_dims, CV_32F);
    block_flow[Y] = cv::Mat::zeros(array_dims, CV_32F);

    //this is the flow at full image size
    full_flow[X] = cv::Mat::zeros(res, CV_32F);
    full_flow[Y] = cv::Mat::zeros(res, CV_32F);

    //this is the flow which might have a 0 border if blocks don't fill the full image space
    pixel_flow[X] = full_flow[X]({0, 0, array_dims.width*block_dims.width, array_dims.height*block_dims.height});
    pixel_flow[Y] = full_flow[Y]({0, 0, array_dims.width*block_dims.width, array_dims.height*block_dims.height});

}

//add a new event to the SAE and record the new event with the
//corresponding block
void zrtFlow::add(int u, int v, double t)
{
    sae.at<double>(v, u) = t;
    blockmap[v*sae.cols+u]->add({u, v});
}

//go through each block and update the list of flow vectors
//update the final flow per pixel
void zrtFlow::update()
{
    //for each block
    for(int by = 0; by < array_dims.height; by++) {
        for(int bx = 0; bx < array_dims.width; bx++) {
            
            //get the block
            auto &b = blocklist[by * array_dims.width + bx];
            //snapshot the event list so we can process in parallel
            b.snap();

            //calculate the connections for each new pixel and add to blocks flow set
            b.updateConnections(sae, con_len, trip_tol);

            //calculate the flow given the connections in
            b.updateFlow(con_buf_min);

                //asign flow to the array
            block_flow[X].at<float>(by, bx) = b.flow.x;
            block_flow[Y].at<float>(by, bx) = b.flow.y;
        }
    }
    //smooth flow - blockFilter on small image (according to zhichao)
    cv::boxFilter(block_flow[X], block_flow[X], -1, {smooth_factor, smooth_factor});
    cv::boxFilter(block_flow[Y], block_flow[Y], -1, {smooth_factor, smooth_factor});

    // cv::GaussianBlur(block_flow[X], block_flow[X], {smooth_factor, smooth_factor}, -1);
    // cv::GaussianBlur(block_flow[Y], block_flow[Y], {smooth_factor, smooth_factor}, -1);

    //resize flow - with linear interpolation (more smoothing)
    cv::resize(block_flow[X], pixel_flow[X], pixel_flow[X].size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(block_flow[Y], pixel_flow[Y], pixel_flow[Y].size(), 0, 0, cv::INTER_LINEAR);
}

cv::Mat zrtFlow::makebgr()
{
    //calculate angle and magnitude
    cv::Mat magnitude, angle;
    cv::cartToPolar(full_flow[X], full_flow[Y], magnitude, angle, true);

    //translate magnitude to range [0;1]
    cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
    magnitude *= 0.05;

    //build hsv image
    cv::Mat _hsv[3];
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnitude;
    cv::merge(_hsv, 3, hsv);

    //convert to BGR
    cv::Mat rgb32;
    cv::cvtColor(hsv, rgb32, cv::COLOR_HSV2BGR);
    rgb32.convertTo(rgb, CV_8UC3, 255);
    return rgb;
}

// ==================================
// zcflow is Zhichao's first version, real-time but lag
// ==================================
const int zcflowBlock::d_coordinate = 2;

void zcflowBlock::initialise(cv::Point2i i)
{
    index = i;
    flow = {0.0,0.0};
    color = {0, 0, 0};
    x_dist.clear();
    y_dist.clear();
}

bool zcflowBlock::block_update_zc(const cv::Mat &sae, int x, int y, cv::Mat &flow_mat, int block_size, cv::Point2i b_index)
{
    point_velocity_zc(sae, x, y, x_dist, y_dist);
    if(x_dist.size() > N && x_dist.size() < 10000) {

        std::sort(x_dist.begin(), x_dist.end());
        std::sort(y_dist.begin(), y_dist.end());

        float temp_x =0, temp_y = 0;
        cv::Point2i temp;
        int counter_x = 0, counter_y = 0;
        bool flag = 1;
        int rows_range = int(y/block_size)+1;
        int cols_range = int(x/block_size)+1;
        // if(rows_range>0 && cols_range>0 && rows_range<23 &&  cols_range<31)
        for(int j=rows_range-1; j<=rows_range+1; j++) {
            for(int i=cols_range-1; i<=cols_range+1; i++) {
                temp.x = fabs(flow_mat.at<cv::Vec2f>(j, i)[0]);
                temp.y = fabs(flow_mat.at<cv::Vec2f>(j, i)[1]);
                if(j==rows_range&&i==cols_range)
                    flag = 0;
                if(fabs(flow_mat.at<cv::Vec2f>(j, i)[0])>0.1 && flag)
                {
                    temp_x += flow_mat.at<cv::Vec2f>(j, i)[0];
                    counter_x += 1;
                }
                if(fabs(flow_mat.at<cv::Vec2f>(j, i)[1])>0.1 && flag)
                {
                    temp_y += flow_mat.at<cv::Vec2f>(j, i)[1];
                    counter_y += 1;
                }
                flag = 1;
            }
        }

        temp_x += (float)(x_dist[x_dist.size()/2]);
        temp_y += (float)(y_dist[y_dist.size()/2]);
        counter_x += 1;
        counter_y += 1;
        temp_x /= counter_x;
        temp_y /= counter_y;
        flow_mat.at<cv::Vec2f>(b_index) = {temp_x, temp_y};
        
        x_dist.clear(); y_dist.clear();

        return true;
    }

    return false;
}

void zcflowBlock::point_velocity_zc(const cv::Mat &sae, int x, int y, std::vector<double> &flow_x, std::vector<double> &flow_y)
{

    for(int j=y - d_coordinate; j <= y + d_coordinate; j++)
        for(int i= x - d_coordinate; i<= x + d_coordinate; i++)
        {   
            if(i!=x or j!=y){
                const double dt12 = sae.at<double>(y, x) - sae.at<double>(j, i);
                if(0 < dt12 && dt12 < dt)
                {
                        //
                        const int m = 2 * i - x;
                        const int n = 2 * j - y;
                        

                        if(0<=n && n<=sae.rows-1 &&0<=m && m<=sae.cols-1)
                        {    
                            const double dt23 = sae.at<double>(j, i)-sae.at<double>(n, m);
                            double error = fabs(1 - dt23/dt12);
                            if(error > tolerance) continue;          //THRESHOLD
                            //valid triplet. calulate the velocity.
                            double invt = 1.0 /  (dt12 + dt23);
                            flow_x.push_back(double(x-m) * invt);
                            flow_y.push_back(double(y-n) * invt);
                        }
                }
            }
            
        }
}

void zcflow::initialise(const cv::Mat_<double> &sae_p, const cv::Mat_<double> &sae_n, int block_size)
{
    this->sae_p = sae_p;
    this->sae_n = sae_n;
    this->block_size = block_size;
    n_blocks = sae_p.size() / block_size;
    n_blocks.width+=boundary_compensation;
    n_blocks.height+=boundary_compensation;
    n_blocks.width+=camera_size_compensation;
    flow_blocks.height = n_blocks.height;
    flow_blocks.width = n_blocks.width;
    flow = cv::Mat::zeros(flow_blocks, CV_32FC2);
    blocks.resize(n_blocks.area());
    flowbgr = cv::Mat::zeros(sae_p.size(), CV_8UC3);
    flow_x = cv::Mat::zeros(sae_p.size(), CV_32F);
    flow_y = cv::Mat::zeros(sae_p.size(), CV_32F);



    for(int y = 0; y < n_blocks.height; y++) {
        for(int x = 0; x < n_blocks.width; x++) {
            blocks[y * n_blocks.width + x].initialise({x, y});
        }
    }

}
void zcflow::update_sae(const cv::Mat_<double> &sae_p, const cv::Mat_<double> &sae_n)
{
    this->sae_p = sae_p;
    this->sae_n = sae_n;
}

void zcflow::clear_blocks()
{
    for(auto &b : blocks){ 
        flow.at<cv::Vec2f>(b.index) = {0.0, 0.0};
        b.x_dist.clear();
        b.y_dist.clear();
    }
}

void zcflow::update(double tic)
{

    for(int y=zcflowBlock::d_coordinate; y< sae_p.rows-zcflowBlock::d_coordinate; y++)
        for(int x=zcflowBlock::d_coordinate; x<sae_p.cols-zcflowBlock::d_coordinate; x++)
        {
            if(sae_p.at<double>(y, x) > toc)
            {
                b_index = blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].index;
                b_index.x += 1;
                b_index.y += 1; 
                blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].block_update_zc(sae_p, x, y, flow, block_size, b_index);
            }
            if(sae_n.at<double>(y, x) > toc)
            {
                b_index = blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].index; 
                b_index.x += 1;
                b_index.y += 1; 
                blocks[int(y/block_size)*n_blocks.width+int(x/block_size)].block_update_zc(sae_n, x, y, flow, block_size, b_index);
            }

        }

    toc = tic;
}

cv::Mat zcflow::makebgr()
{

    flow_ = flow(cv::Range(1, flow.rows-1), cv::Range(1,flow.cols-1));


    cv::split(flow_, xy);

    //calculate angle and magnitude
    cv::Mat magnitude, angle;
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

    //translate magnitude to range [0;1]
    cv::threshold(magnitude, magnitude, 20, 20, cv::THRESH_TRUNC);
    magnitude *= 0.05;

    //build hsv image
    cv::Mat _hsv[3], hsv;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnitude;
    cv::merge(_hsv, 3, hsv);

    //convert to BGR
    cv::Mat small;
    cv::cvtColor(hsv, small, cv::COLOR_HSV2BGR);
    small.convertTo(small, CV_8UC3, 255);
    cv::resize(small, flowbgr, sae_p.size(), 0.0, 0.0, cv::INTER_LINEAR);
    return flowbgr;
} 
}