#include "iCub/vDraw.h"

void vDraw::format(cv::Mat &canvas)
{
    cv::Mat correctChannels(canvas.size(), CV_8UC3);
    correctChannels.convertTo(CV_8UC3);
    if(canvas.type() == CV_8U) {
        cv::cvtColor(canvas, correctChannels, CV_GRAY2BGR);
    } else {
        correctChannels = canvas; //shallow copy
    }

    cv::flip(correctChannels, correctChannels, 0);

    cv::resize(correctChannels, imageOnThePort,
               imageOnThePort.size(), 0, 0, cv::INTER_NEAREST);
}

std::string addressDraw::getTag()
{
    return string("AE");
}

addressDraw::draw(cv::Mat &canvas, const emorph::vQueue &eSet, int X, int Y)
{

    //first we make sure the canvas is in the correct format
    if(canvas.channels() == 1) {
        canvas.convertTo(canvas, CV_8UC3);
    }
    cv::resize(canvas, canvas, )
    cv::Mat canvas(X, Y, CV_8UC3);
    canvas.setTo(128);


    int mass = 180;
    //std::cout << "Drawing " << eSet.size() << std::endl;
    emorph::vQueue::iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(aep) {
            cv::Vec3b cpc = canvas.at<cv::Vec3b>(aep->getX(), aep->getY());

            if(aep->getPolarity())
            {
                if(cpc.val[1] > 255 - mass) cpc.val[1] = 255;
                else cpc.val[1] += mass;
            }
            else
            {
                if(cpc.val[2] > 255 - mass) cpc.val[2] = 255;
                else cpc.val[2] += mass;
            }

            canvas.at<cv::Vec3b>(aep->getX(), aep->getY()) = cpc;
        }

    }

    return canvas;
}
