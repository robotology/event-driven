#include <iCub/emorph/all.h>
#include <string>
#include <opencv2/opencv.hpp>

/*!
 * \brief The vFrame class
 */
class vDraw {

private:

    void format(cv::Mat &canvas);

public:

    virtual cv::Mat draw(cv::Mat &canvas, const emorph::vQueue &eSet) = 0;
    std::string getTag() = 0;


};

class addressDraw : public vDraw {};

