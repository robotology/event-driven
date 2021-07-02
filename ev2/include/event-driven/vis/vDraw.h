/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
 *           chiara.bartolozzi@iit.it
 *           massimiliano.iacono@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace ev { namespace vis {


template<typename T> inline void drawGrey(cv::Mat_<unsigned char> &image, T)
{
    image.at(T.y, T.x) = T.p ? 255 : 0;
}

/**
 * @brief The vDraw class is the base class from which all vDrawers should
 * inherit. It contains the draw and getTag functions which must be overloaded,
 * and the sensor size for reference.
 */
template <typename T> class vDraw {

protected:

    static const cv::Vec3b aqua {151, 174, 6};
    static const cv::Vec3b violet  {180, 10, 155};
    static const cv::Vec3b orange {9, 111, 255};
    static const cv::Vec3b lime   {9, 250, 222};
    static const cv::Vec3b white{255, 255, 255};
    static const cv::Vec3b black  {0,   0,   0};
    static const cv::Vec3b red  {0,   0,   255};

public:

    vDraw() : Xlimit(304), Ylimit(240), flip(false) {}

    void setResolution(int height, int width)
    {
        res = cv::Size(width, height);
    }

    void setFlip(bool flip)
    {
        this->flip = flip;
    }

    virtual void initialise() {}

    virtual void resetImage(cv::Mat &image)
    {
        if (canvas.empty())
            canvas = cv::Mat(res, CV_8UC3);
        image.setTo(255);
    }

    inline virtual void draw(cv::Mat &image, T event) = 0;

    //template<class I> typename enable_if<is_same<typename iterator_traits<I>::value_type, T>>::type draw(cv::Mat &image, I start, I end);
};

class isoDraw : public vDraw<AE> {

protected:

    //angles
    double thetaY;
    double thetaX;
    double CY, SY;
    double CX, SX;

    double ts_to_axis;
    int Zlimit;
    int imagewidth;
    int imageheight;
    int imagexshift;
    int imageyshift;

    //private functions
    inline void pttr(int &x, int &y, int &z) {
        // we want a negative rotation around the y axis (yaw)
        // a positive rotation around the x axis (pitch) (no roll)
        // the z should always be negative values.
        // the points need to be shifted across by negligble amount
        // the points need to be shifted up by (x = max, y = 0, ts = 0 rotation)

        int xmod = x*CY + z*SY + 0.5; // +0.5 rounds rather than floor
        int ymod = y*CX - SX*(-x*SY + z*CY) + 0.5;
        int zmod = y*SX + CX*(-x*SY + z*CY) + 0.5;
        x = xmod; y = ymod; z = zmod;
    }

    //image with warped square drawn
    cv::Mat baseimage;

public:

    void initialise();
    inline void draw(cv::Mat &image, AE &event);

};

class addressDraw : public vDraw<AE> {

public:
    virtual void draw(cv::Mat &image, AE &event);
};

class imuDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class rasterDraw : public vDraw {

protected:

    float time_scaler;
    unsigned int num_neurons;

public:

    virtual void initialise()
    {
        Xlimit = 1024;
        Ylimit = 1024;

        num_neurons = 1;
        time_scaler = (double) Xlimit / max_window;
    }

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class cochleaDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class flowDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class interestDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class clusterDraw : public vDraw {

protected:

    std::map<int, ev::event<ev::GaussianAE>> persistance;
    int stagnantCount;

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class isoInterestDraw : public isoDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};


} }  //namespace ev::vis::

#endif


