#include "aggregation.hpp"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace emorph::reco;

aggregation::aggregation()
{
}

aggregation::aggregation(std::string &_f, BufferedPort< yarp::sig::ImageOf <yarp::sig::PixelMono16> > *_port)
:imgFile(_f)
{
    port=_port;
}

aggregation::~aggregation()
{
}

void aggregation::onRead(objDistBuffer &_obj)
{
    if(!_obj.getEye())
    {
        std::cout << "[Aggregation] Left set of distances received" << std::endl;
        szLeft=_obj.getSize();

        std::cout << "[Aggregation] Number of left distance pos/neg: " << szLeft << std::endl;

        posLeftEye.resize(szLeft/2);
        memcpy(posLeftEye.data(), _obj.getBuffer(), (szLeft/2)*sizeof(double));

        std::cout << "[Aggregation] First pos dist: " << *(_obj.getBuffer()) << ", equivalent in the vector: " << posLeftEye(0) << std::endl;

        negLeftEye.resize(szLeft/2);
        memcpy(negLeftEye.data(), _obj.getBuffer()+(szLeft/2), (szLeft/2)*sizeof(double));

        std::cout << "[Aggregation] First neg dist: " << *(_obj.getBuffer()+szLeft/2) << ", equivalent in the vector: " << negLeftEye(0) << std::endl;
        rcvLeft=true;
#ifdef _DEBUG
    std::cout << "posLeftEye: " << std::endl;
    printVector(&posLeftEye);
    std::cout << "negLeftEye: " << std::endl;
    printVector(&negLeftEye);
#endif
    }
    else
    {
        std::cout << "[Aggregation] Right set of distances received" << std::endl;
        szRight=_obj.getSize();
        std::cout << "[Aggregation] Number of right distance pos/neg: " << szRight << std::endl;

        posRightEye.resize(szRight/2);
        memcpy(posRightEye.data(), _obj.getBuffer(), (szRight/2)*sizeof(double));

        std::cout << "[Aggregation] First pos dist: " << *(_obj.getBuffer()) << ", equivalent in the vector: " << posRightEye(0) << std::endl;

        negRightEye.resize(szRight/2);
        memcpy(negRightEye.data(), _obj.getBuffer()+(szRight/2), (szRight/2)*sizeof(double));

        std::cout << "[Aggregation] First neg dist: " << *(_obj.getBuffer()+szRight/2) << ", equivalent in the vector: " << negRightEye(0) << std::endl;
        rcvRight=true;
#ifdef _DEBUG
    std::cout << "posRightEye: " << std::endl;
    printVector(&posRightEye);
    std::cout << "negRightEye: " << std::endl;
    printVector(&negRightEye);
#endif
    }
    if(rcvLeft && rcvRight)
    {
        std::cout << "[Aggregation] Both set of distances received" << std::endl;
        if(szLeft==szRight)
        {
            aggregate();
            rcvLeft=false;
            rcvRight=false;
        }
        else
            std::cout << "[Aggregation] Error: the data has not the size size for the both eyes" << std::endl;
    }
}

void aggregation::aggregate()
{
    posLeftEye/=gsl_stats_max(posLeftEye.data(), 1, szLeft/2);
    negLeftEye/=gsl_stats_max(negLeftEye.data(), 1, szLeft/2);
    posRightEye/=gsl_stats_max(posRightEye.data(), 1, szRight/2);
    negRightEye/=gsl_stats_max(negRightEye.data(), 1, szRight/2);

#ifdef _DEBUG
    std::cout << "Normalized: " << std::endl;

    std::cout << "posLeftEye: " << std::endl;
    printVector(&posLeftEye);
    std::cout << "negLeftEye: " << std::endl;
    printVector(&negLeftEye);
    std::cout << "posRightEye: " << std::endl;
    printVector(&posRightEye);
    std::cout << "negRightEye: " << std::endl;
    printVector(&negRightEye);
#endif

    posLeftEye=(1.0-posLeftEye)/sum(posLeftEye);
    negLeftEye=(1.0-negLeftEye)/sum(negLeftEye);
    posRightEye=(1.0-posRightEye)/sum(posRightEye);
    negRightEye=(1.0-negRightEye)/sum(negRightEye);

#ifdef _DEBUG
    std::cout << "Inverted: " << std::endl;

    std::cout << "posLeftEye: " << std::endl;
    printVector(&posLeftEye);
    std::cout << "negLeftEye: " << std::endl;
    printVector(&negLeftEye);
    std::cout << "posRightEye: " << std::endl;
    printVector(&posRightEye);
    std::cout << "negRightEye: " << std::endl;
    printVector(&negRightEye);
#endif

    Vector bothEye(szLeft/2);
    Vector winners(szLeft/2, 0.0);

    bothEye=posLeftEye*negLeftEye*posRightEye*negRightEye;
#ifdef _DEBUG
    std::cout << "bothEye: " << std::endl;
    printVector(&bothEye);
#endif    
    size_t index=gsl_stats_max_index (posLeftEye.data(), 1, szLeft/2);
    winners(index)++;
    index=gsl_stats_max_index (negLeftEye.data(), 1, szLeft/2);
    winners(index)++;
    index=gsl_stats_max_index (posRightEye.data(), 1, szRight/2);
    winners(index)++;
    index=gsl_stats_max_index (negRightEye.data(), 1, szRight/2);
    winners(index)++;
#ifdef _DEBUG
    std::cout << "winners before: " << std::endl;
    printVector(&winners);
#endif    

    winners=bothEye*(winners/4);
#ifdef _DEBUG
    std::cout << "winners after: " << std::endl;
    printVector(&winners);
#endif    

    size_t winner=gsl_stats_max_index (winners.data(), 1, szLeft/2);
    std::cout << "Index of the recognize objct: " << winner << std::endl;
}

double aggregation::sum(Vector &_vec)
{
    std::cout << "[Aggregation] sum()" << std::endl;
    double res=0;
    double *_data=_vec.data();
    size_t szVec=_vec.size();
    for(int i=0; i<szVec; ++i){
        res+=*(_data+i);
        std::cout << "\t" << res << std::endl;
    }
    return res;
}

int aggregation::loadPictures()
{
/*    knowledgeImg=new cv::Mat*[knowledgeSize];

    //size_t sasPosition=knowledgeFileList.find_last_of("/\\");
    size_t periodPosition=knowledgeFileList.find_last_of(".");

    //if(sasPosition==string::npos)
    if(periodPosition==string::npos)
        return -1;
    //string dir=knowledgeFileList.substr(0, sasPosition+1);
    stringstream file2load;
    //file2load << dir << "pictures.list";
    file2load << knowledgeFileList.substr(periodPosition+1) << ".img";
    cout << "File containing the img location " << file2load << endl;
    unsigned int imgIndex=0;
    string imFile;
    ifstream fd;
    fd.open(file2load.str().c_str());
    while(fd.good() && imgIndex<knowledgeSize)
    {
        getline(fd, imFile);
        knowledgeImg[imgIndex]=new cv::Mat(128, 128, CV_8UC1);
        cout << "[recognition] current img to load: " << imFile << endl;
        *knowledgeImg[imgIndex]=cv::imread(imFile, 1);
        //knowledgeImg[imgIndex]->convertTo(*knowledgeImg[imgIndex], 16);
        //cv::imshow("img preview", *knowledgeImg[imgIndex]);
        //cv::waitKey(0);
        ++imgIndex;
    }
//    cout << "Close the file..." << endl;
    fd.close();
*/
}

void aggregation::sendImg(int &_index)
{
/*
    size_t respos=gsl_stats_min_index(distpos, 1, sznposh);
    size_t resneg=gsl_stats_min_index(distneg, 1, sznnegh);

    if(respos==resneg)
    {
        yarp::sig::ImageOf<yarp::sig::PixelMono16>& resImg=outPort->prepare();
        //IplImage iplImg=*knowledgeImg[respos];
        cv::imshow("Original img", *knowledgeImg[respos]); cv::waitKey(0);
        //cv::Mat bgrMat(128, 128, CV_8UC3);
        cv::Mat bgrMat = cv::Mat::ones(128, 128, CV_8UC3)*126;
        cv::imshow("Uninitialized bgr img", bgrMat); cv::waitKey(0);
        cv::cvtColor(*knowledgeImg[respos], bgrMat, CV_GRAY2BGR, 3);
        cv::imshow("Initialized bgr img", bgrMat); cv::waitKey(0);
        //IplImage *iplImg = cvCreateImage(cvSize(128,128), IPL_DEPTH_8U, 1);
        //iplImg->imageData = (char *) (*knowledgeImg[respos]).data;

        IplImage *iplout = cvCreateImage(cvSize(128,128), IPL_DEPTH_8U, 3); //usually RGB image are not floating point 
        iplout->imageData = (char *) bgrMat.data;

        cout << "depth: " << iplout->depth << endl
            << "IPL_DEPTH_8U: " << IPL_DEPTH_8U << endl
            << "IPL_DEPTH_8S: " << IPL_DEPTH_8S << endl
            << "IPL_DEPTH_16U: " << IPL_DEPTH_16U << endl
            << "IPL_DEPTH_16S: " << IPL_DEPTH_16S << endl
            << "IPL_DEPTH_32S: " << IPL_DEPTH_32S << endl
            << "IPL_DEPTH_32F: " << IPL_DEPTH_32F << endl
            << "IPL_DEPTH_64F: " << IPL_DEPTH_64F << endl;

        resImg.wrapIplImage(iplout);
        outPort->write();

        //cv::imshow("img preview", *knowledgeImg[respos]);
        //cv::waitKey(0);
    }
    else
        cout << "Uncertain results" << endl;*/
}
#ifdef _DEBUG
void aggregation::printVector(yarp::sig::Vector* _vec)
{
    //std::cout << "vec: ";
    double *dat=_vec->data();
    for(unsigned int e=0; e<_vec->size(); e++)
        std::cout << *(dat+e) << " ";
    std::cout << std::endl;
}
#endif
