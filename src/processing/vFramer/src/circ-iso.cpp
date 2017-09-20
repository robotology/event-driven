#include "vDraw.h"
#include "yarp/os/all.h"

using namespace ev;

const std::string isoCircDraw::drawtype = "ISO-CIRC";
std::string isoCircDraw::getDrawType()
{
    return isoCircDraw::drawtype;
}
std::string isoCircDraw::getEventType()
{
    return ev::GaussianAE::tag;
}
void isoCircDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);
    cv::Scalar red = CV_RGB(255, 0, 0);

    if(eSet.empty()) return;

    if(image.rows != imageheight || image.cols != imagewidth) {
        yWarning() << "Could not draw isoCircDraw. Please draw ISO first";
        return;
    }

    auto v = is_event<GaussianAE>(eSet.back());
    //if(v->x < 0 || v->x >= Xlimit || v->y < 0 || v->y >= Ylimit) continue;

    int px1 = v->x;
    int py1 = v->y;
    int pz1 = 0;

    if(flip) {
        px1 = Xlimit - 1 - px1;
        py1 = Ylimit - 1 - py1;
    }

    int px2 = px1;
    int py2 = py1;
    int pz2 = Zlimit * (v->sigy / ev::vtsHelper::max_stamp) + 0.5;

    pttr(px1, py1, pz1);
    pttr(px2, py2, pz2);

    px1 += imagexshift;
    py1 += imageyshift;
    px2 += imagexshift;
    py2 += imageyshift;

    if(px1 < 0) px1 = 0; if(px1 >= imagewidth) px1 = imagewidth -1;
    if(py1 < 0) py1 = 0; if(py1 >= imageheight) py1 = imageheight -1;
    if(px2 < 0) px2 = 0; if(px2 >= imagewidth) px2 = imagewidth -1;
    if(py2 < 0) py2 = 0; if(py2 >= imageheight) py2 = imageheight -1;

    cv::Point p1(px1, py1);
    cv::Point p2(px2, py2);

    if(v->polarity)
        cv::line(image, p1, p2, blue, 2.0);
    else
        cv::line(image, p1, p2, red, 2.0);

}
