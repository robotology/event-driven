#include "event-driven/vis/draw.h"

namespace ev {

pixelShifter::pixelShifter() {
    setRotation(20.0, 40.0);
    setShift(0, 0, 1.0);
}

void pixelShifter::setRotation(double pitch, double yaw) {
    thetaX = pitch * 3.14 / 180.0;  //PITCH
    thetaY = yaw * 3.14 / 180.0;    //YAW

    CY = cos(thetaY);
    SY = sin(thetaY);
    CX = cos(thetaX);
    SX = sin(thetaX);
}

void pixelShifter::setShift(int xoffset, int yoffset, double tsoffset) {
    xshift = xoffset;
    yshift = yoffset;
    ts_scaler = tsoffset;
}

void pixelShifter::pttr(int &x, int &y, double &z) {
    // we want a negative rotation around the y axis (yaw)
    // a positive rotation around the x axis (pitch) (no roll)
    // the z should always be negative values.
    // the points need to be shifted across by negligble amount
    // the points need to be shifted up by (x = max, y = 0, ts = 0 rotation)
    z = z * ts_scaler;
    int xmod = x * CY + z * SY + 0.5;  // +0.5 rounds rather than floor
    int ymod = y * CX - SX * (-x * SY + z * CY) + 0.5;
    //int zmod = y*SX + CX*(-x*SY + z*CY) + 0.5;
    x = xmod + xshift;
    y = ymod + yshift;  //z = zmod;
}

pixelShifter drawISOBase(int height, int width, int period, cv::Mat &baseimage)
{
    int Xlimit = width;
    int Ylimit = height;
    int Zlimit = width * 3;
    double ts_to_axis = (double)Zlimit / period;

    pixelShifter pr;

    //the following calculations make the assumption of a negative yaw and
    //a positive pitch
    int x, y; double z;
    int maxx = 0, maxy = 0, miny = Ylimit, minx = Xlimit;
    for(int xi = 0; xi <= Xlimit; xi+=Xlimit) {
        for(int yi = 0; yi <= Ylimit; yi+=Ylimit) {
            for(double zi = 0; zi <= Zlimit; zi+=Zlimit) {
                x = xi; y = yi; z = zi; pr.pttr(x, y, z);
                maxx = std::max(maxx, x);
                maxy = std::max(maxy, y);
                minx = std::min(minx, x);
                miny = std::min(miny, y);
            }
        }
    }


    int imagexshift = -minx + 10;
    int imageyshift = -miny + 10;
    pr.setShift(imagexshift, imageyshift, ts_to_axis);

    int imagewidth = maxx + imagexshift + 10;
    int imageheight = maxy + imageyshift + 10;

    baseimage = cv::Mat(imageheight, imagewidth, CV_8UC3);
    baseimage.setTo(0);

    

    //cv::putText(baseimage, std::string("X"), cv::Point(100, 100), 1, 0.5, CV_RGB(0, 0, 0));

    cv::Scalar invertedtextc = CV_RGB(125, 125, 125);
    cv::Vec3b invertedaxisc = cv::Vec3b(255, 255, 255);
    cv::Vec3b invertedframec = cv::Vec3b(125, 125, 125);

    for(int xi = 0; xi < Xlimit; xi++) {
        x = xi; y = 0; z = 0; pr.pttr(x, y, z);
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        x = xi; y = Ylimit; z = 0; pr.pttr(x, y, z);
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        if(xi == Xlimit / 2) {
            cv::putText(baseimage, std::string("x"), cv::Point(x-10, y+10),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }
    }

    for(int yi = 0; yi <= Ylimit; yi++) {
        x = 0; y = yi; z = 0; pr.pttr(x, y, z);
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        if(yi == Ylimit / 2) {
            cv::putText(baseimage, std::string("y"), cv::Point(x-10, y+10),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }
        x = Xlimit; y = yi; z = 0; pr.pttr(x, y, z);
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

    }

    unsigned int tsi;
    for(tsi = 0; tsi < (unsigned int)(period*0.3); tsi++) {

        x = Xlimit; y = Ylimit; z = tsi; pr.pttr(x, y, z);
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

        if(tsi == (unsigned int)(period *0.15)) {
            cv::putText(baseimage, std::string("t"), cv::Point(x, y+12),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }

    }

    // for(int i = 0; i < 14; i++) {

    //     x = Xlimit-i/2; y = Ylimit; z = tsi-i; pr.pttr(x, y, z);
    //     baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

    //     x = Xlimit+i/2; y = Ylimit; z = tsi-i; pr.pttr(x, y, z);
    //     baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
    // }

    for(tsi = ev::vtsscaler / 10.0;
        tsi < (unsigned int)period;
        tsi += ev::vtsscaler / 10.0) {

        int zc = tsi + 0.5;

        for(int xi = 0; xi < Xlimit; xi++) {
            x = xi; y = 0; z = zc; pr.pttr(x, y, z);
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;
            x = xi; y = Ylimit; z = zc; pr.pttr(x, y, z);
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;
        }

        for(int yi = 0; yi <= Ylimit; yi++) {
            x = 0; y = yi; z = zc; pr.pttr(x, y, z);
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;
            x = Xlimit; y = yi; z = zc; pr.pttr(x, y, z);
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;

        }

    }

    return pr;
}

}