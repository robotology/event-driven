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

#ifndef __vDraw__
#define __vDraw__

#include <event-driven/all.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace ev {

/**
 * @brief The vDraw class is the base class from which all vDrawers should
 * inherit. It contains the draw and getTag functions which must be overloaded,
 * and the sensor size for reference.
 */
class vDraw {

protected:

    int Xlimit;
    int Ylimit;
    unsigned int display_window;
    unsigned int max_window;
    bool flip;
    


    cv::Vec3b aqua {151, 174, 6};
    cv::Vec3b violet  {180, 10, 155};
    cv::Vec3b orange {9, 111, 255};
    cv::Vec3b lime   {9, 250, 222};
    cv::Vec3b white{255, 255, 255};
    cv::Vec3b black  {0,   0,   0};
    cv::Vec3b red  {0,   0,   255};

public:

    vDraw() : Xlimit(304), Ylimit(240), flip(false)

    {
        display_window = 0.1*ev::vtsHelper::vtsscaler;
        max_window = 0.5*ev::vtsHelper::vtsscaler;
        // display_window = ev::vtsHelper::vtsscaler;
        // max_window = 5*ev::vtsHelper::vtsscaler;
    }

    virtual ~vDraw() {}

    ///
    /// \brief setLimits sets the maximum possible values of the position of
    /// events that could be drawn (mostly governed by the sensor size).
    /// \param Xlimit is the maximum x value (width)
    /// \param Ylimit is the maximium y value (height)
    ///
    void setRetinaLimits(int Xlimit, int Ylimit)
    {
        this->Xlimit = Xlimit;
        this->Ylimit = Ylimit;
    }

    void setTemporalLimits(unsigned int display_window,
                           unsigned int max_window)
    {
        this->display_window = display_window;
        this->max_window = max_window;
    }

    void setFlip(bool flip)
    {
        this->flip = flip;
    }

    virtual void initialise() {}

    virtual void resetImage(cv::Mat &image)
    {
        if(image.empty())
            image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    ///
    /// \brief draw takes an image and overlays the new visualisation textures
    /// \param canvas is the image which may or may not yet exist
    /// \param eSet is the set of events which could possibly be drawn
    ///
    virtual void draw(cv::Mat &canvas, const ev::vQueue &eSet, int vTime) = 0;

    ///
    /// \brief getTag returns the unique code for this drawing method. The
    /// arguments given on the command line must match this code exactly
    /// \return the tag code
    ///
    virtual std::string getDrawType() = 0;

    virtual std::string getEventType() = 0;


};

class isoDraw : public vDraw {

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

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class addressDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class skinDraw : public vDraw {
private:

std::map<int,std::tuple<int, int>> tmap{};//,tmap_ref{};
int scaling =20;
int xoffset = 80;
int yoffset = 120;
//Luca: create parent class for events and sample based skin plots ?
cv::Mat baseimage;

public:
    
    void initialise(){

/* Luca :builds the taxel map 
Hardcoding but could be interesting maybe to read it from file?*/
        Xlimit = 700;
        Ylimit = 800;

        tmap={{0,{24, 9}},{1,{24, 7}},{2,{23, 8}},{3,{23, 10}},{4,{22, 9}},{5,{21, 10}},{7,{22, 11}},{8,{23, 12}},{9,{24, 13}},{11,{24, 11}},{16,{22, 17}},{17,{23, 16}},
        {18,{22, 15}},{19,{21, 16}},{20,{21, 14}},{21,{20, 13}},{23,{20, 15}},{24,{20, 17}},{25,{20, 19}},{27,{21, 18}},{32,{24, 21}},{33,{24, 19}},
        {34,{23, 20}},{35,{23, 22}},{36,{22, 21}},{37,{21, 22}},{39,{22, 23}},{40,{23, 24}},{41,{24, 25}},{43,{24, 23}},{48,{2, 11}},{49,{2, 13}},
        {50,{3, 12}},{51,{3, 10}},{52,{4, 11}},{53,{5, 10}},{55,{4, 9}},{56,{3, 8}},{57,{2, 7}},{59,{2, 9}},{64,{4, 3}},{65,{3, 4}},
        {66,{4, 5}},{67,{5, 4}},{68,{5, 6}},{69,{6, 7}},{71,{6, 5}},{72,{6, 3}},{73,{6, 1}},{75,{5, 2}},{80,{8, 5}},{81,{8, 7}},
        {82,{9, 6}},{83,{9, 4}},{84,{10, 5}},{85,{11, 4}},{87,{10, 3}},{88,{9, 2}},{89,{8, 1}},{91,{8, 3}},{96,{5, 18}},{97,{6, 19}},
        {98,{6, 17}},{99,{5, 16}},{100,{6, 15}},{101,{6, 13}},{103,{5, 14}},{104,{4, 15}},{105,{3, 16}},{107,{4, 17}},{112,{4, 23}},{113,{5, 22}},
        {114,{4, 21}},{115,{3, 22}},{116,{3, 20}},{117,{2, 19}},{119,{2, 21}},{120,{2, 23}},{121,{2, 25}},{123,{3, 24}},{128,{11, 24}},{129,{12, 25}},
        {130,{12, 23}},{131,{11, 22}},{132,{12, 21}},{133,{12, 19}},{135,{11, 20}},{136,{10, 21}},{137,{9, 22}},{139,{10, 23}},{144,{8, 17}},{145,{8, 19}},
        {146,{9, 18}},{147,{9, 16}},{148,{10, 17}},{149,{11, 16}},{151,{10, 15}},{152,{9, 14}},{153,{8, 13}},{155,{8, 15}},{160,{10, 9}},{161,{9, 10}},
        {162,{10, 11}},{163,{11, 10}},{164,{11, 12}},{165,{12, 13}},{167,{12, 11}},{168,{12, 9}},{169,{12, 7}},{171,{11, 8}},{176,{15, 8}},{177,{14, 7}},
        {178,{14, 9}},{179,{15, 10}},{180,{14, 11}},{181,{14, 13}},{183,{15, 12}},{184,{16, 11}},{185,{17, 10}},{187,{16, 9}},{192,{17, 18}},{193,{18, 19}},
        {194,{18, 17}},{195,{17, 16}},{196,{18, 15}},{197,{18, 13}},{199,{17, 14}},{200,{16, 15}},{201,{15, 16}},{203,{16, 17}},{208,{16, 23}},{209,{17, 22}},
        {210,{16, 21}},{211,{15, 22}},{212,{15, 20}},{213,{14, 19}},{215,{14, 21}},{216,{14, 23}},{217,{14, 25}},{219,{15, 24}},{224,{16, 3}},{225,{15, 4}},
        {226,{16, 5}},{227,{17, 4}},{228,{17, 6}},{229,{18, 7}},{231,{18, 5}},{232,{18, 3}},{233,{18, 1}},{235,{17, 2}},{240,{21, 2}},{241,{20, 1}},
        {242,{20, 3}},{243,{21, 4}},{244,{20, 5}},{245,{20, 7}},{247,{21, 6}},{248,{22, 5}},{249,{23, 4}},{251,{22, 3}}};
        

        baseimage = cv::Mat(cv::Size(Xlimit, Ylimit), CV_8UC3);
        baseimage.setTo(255);

        int x;
        int y;
        for (auto it = tmap.begin(); it != tmap.end(); it++)
        {
            x = xoffset + scaling* std :: get<0>(it->second);
            y =  Ylimit - (yoffset +scaling* std :: get<1>(it->second));
            cv::Point centr_all(x, y);
            cv::circle(baseimage, centr_all, 10, black,1, CV_AA);
        }

        };

        

    virtual void resetImage(cv::Mat &image)
    {
        baseimage.copyTo(image);
    }
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class skinsampleDraw : public vDraw {
private:

std::map<int,std::tuple<int, int>> tmap{};//,tmap_ref{};
cv::Mat baseimage;

int scaling =20;
int xoffset = 80;
int yoffset = 120;

public:
  void initialise(){
        Xlimit = 700;
        Ylimit = 800;


        tmap= {{0,{24, 9}},{1,{24, 7}},{2,{23, 8}},{3,{23, 10}},{4,{22, 9}},{5,{21, 10}},{7,{22, 11}},{8,{23, 12}},{9,{24, 13}},{11,{24, 11}},{16,{22, 17}},{17,{23, 16}},
        {18,{22, 15}},{19,{21, 16}},{20,{21, 14}},{21,{20, 13}},{23,{20, 15}},{24,{20, 17}},{25,{20, 19}},{27,{21, 18}},{32,{24, 21}},{33,{24, 19}},
        {34,{23, 20}},{35,{23, 22}},{36,{22, 21}},{37,{21, 22}},{39,{22, 23}},{40,{23, 24}},{41,{24, 25}},{43,{24, 23}},{48,{2, 11}},{49,{2, 13}},
        {50,{3, 12}},{51,{3, 10}},{52,{4, 11}},{53,{5, 10}},{55,{4, 9}},{56,{3, 8}},{57,{2, 7}},{59,{2, 9}},{64,{4, 3}},{65,{3, 4}},
        {66,{4, 5}},{67,{5, 4}},{68,{5, 6}},{69,{6, 7}},{71,{6, 5}},{72,{6, 3}},{73,{6, 1}},{75,{5, 2}},{80,{8, 5}},{81,{8, 7}},
        {82,{9, 6}},{83,{9, 4}},{84,{10, 5}},{85,{11, 4}},{87,{10, 3}},{88,{9, 2}},{89,{8, 1}},{91,{8, 3}},{96,{5, 18}},{97,{6, 19}},
        {98,{6, 17}},{99,{5, 16}},{100,{6, 15}},{101,{6, 13}},{103,{5, 14}},{104,{4, 15}},{105,{3, 16}},{107,{4, 17}},{112,{4, 23}},{113,{5, 22}},
        {114,{4, 21}},{115,{3, 22}},{116,{3, 20}},{117,{2, 19}},{119,{2, 21}},{120,{2, 23}},{121,{2, 25}},{123,{3, 24}},{128,{11, 24}},{129,{12, 25}},
        {130,{12, 23}},{131,{11, 22}},{132,{12, 21}},{133,{12, 19}},{135,{11, 20}},{136,{10, 21}},{137,{9, 22}},{139,{10, 23}},{144,{8, 17}},{145,{8, 19}},
        {146,{9, 18}},{147,{9, 16}},{148,{10, 17}},{149,{11, 16}},{151,{10, 15}},{152,{9, 14}},{153,{8, 13}},{155,{8, 15}},{160,{10, 9}},{161,{9, 10}},
        {162,{10, 11}},{163,{11, 10}},{164,{11, 12}},{165,{12, 13}},{167,{12, 11}},{168,{12, 9}},{169,{12, 7}},{171,{11, 8}},{176,{15, 8}},{177,{14, 7}},
        {178,{14, 9}},{179,{15, 10}},{180,{14, 11}},{181,{14, 13}},{183,{15, 12}},{184,{16, 11}},{185,{17, 10}},{187,{16, 9}},{192,{17, 18}},{193,{18, 19}},
        {194,{18, 17}},{195,{17, 16}},{196,{18, 15}},{197,{18, 13}},{199,{17, 14}},{200,{16, 15}},{201,{15, 16}},{203,{16, 17}},{208,{16, 23}},{209,{17, 22}},
        {210,{16, 21}},{211,{15, 22}},{212,{15, 20}},{213,{14, 19}},{215,{14, 21}},{216,{14, 23}},{217,{14, 25}},{219,{15, 24}},{224,{16, 3}},{225,{15, 4}},
        {226,{16, 5}},{227,{17, 4}},{228,{17, 6}},{229,{18, 7}},{231,{18, 5}},{232,{18, 3}},{233,{18, 1}},{235,{17, 2}},{240,{21, 2}},{241,{20, 1}},
        {242,{20, 3}},{243,{21, 4}},{244,{20, 5}},{245,{20, 7}},{247,{21, 6}},{248,{22, 5}},{249,{23, 4}},{251,{22, 3}}};
        

        baseimage = cv::Mat(cv::Size(Xlimit, Ylimit), CV_8UC3);
        baseimage.setTo(255);

        int x;
        int y;
        for (auto it = tmap.begin(); it != tmap.end(); it++)
        {
            x = xoffset + scaling* std :: get<0>(it->second);
            y =  Ylimit - (yoffset +scaling* std :: get<1>(it->second));
            cv::Point centr_all(x, y);
            cv::circle(baseimage, centr_all, 10, black,1, CV_AA);
        }
    };



    virtual void resetImage(cv::Mat &image)
    {
        baseimage.copyTo(image);
    }
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class taxel : public vDraw{
    protected:
    static float T;
};

class taxelsampleDraw : public taxel {

    
    public:
    virtual void resetImage(cv::Mat &image){
        if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
        }
    }
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class taxeleventDraw : public taxel {
   
    public:
    virtual void resetImage(cv::Mat &image){
        if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
        }
    }
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

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




} //namespace ev::

#endif


