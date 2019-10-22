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

class vDraw;

/**
 * @brief createDrawer returns an instance of a drawer that matches the tag
 * specified
 * @param tag is the code of the drawer
 * @return a pointer to a drawer on success. 0 if no appropiate drawer was
 * found
 */
vDraw * createDrawer(std::string tag);

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

class accDraw : public vDraw {

public:

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

class grayDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class skinDraw : public vDraw {
private:
//Luca: do we need method to print it?
std::list<std::tuple<int, int>> t_map{};

//Luca: create parent class for events and sample based skin plots ?

public:
    
    void initialise(){

/* Luca :builds the taxel map 
Hardcoding but could be interesting maybe to read it from file?*/
        Xlimit = 600;
        Ylimit =600;
        t_map={{24, 7},{24, 9},{24, 11},{24, 13},{23, 12},{23, 10},{23, 8},{22, 11},{22, 9},{21, 10},
        {24, 19},{24, 21},{24, 23},{24, 25},{23, 24},{23, 22},{23, 20},{22, 23},{22, 21},{21, 22},
        {23, 16},{22, 17},{22, 15},{21, 18},{21, 16},{21, 14},{20, 19},{20, 17},{20, 15},{20, 13},
        {23, 4},{22, 5},{22, 3},{21, 4},{21, 6},{21, 2},{20, 7},{20, 5},{20, 3},{20, 1},
        {18, 19},{18, 17},{18, 15},{18, 13},{17, 18},{17, 16},{17, 14},{16, 17},{16, 15},{15, 16},
        {18, 7},{18, 5},{18, 3},{18, 1},{17, 6},{17, 4},{17, 2},{16, 5},{16, 3},{15, 4},
        {17, 10},{16, 11},{16, 9},{15, 12},{15, 10},{15, 8},{14, 13},{14, 11},{14, 9},{14, 7},
        {17, 22},{16, 23},{16, 21},{15, 24},{15, 22},{15, 20},{14, 25},{14, 23},{14, 21},{14, 19},
        {12, 19},{12, 21},{12, 23},{12, 25},{11, 24},{11, 22},{11, 20},{10, 23},{10, 21},{9, 22},
        {12, 13},{12, 11},{12, 9},{12, 7},{11, 12},{11, 10},{11, 8},{10, 11},{10, 9},{9, 10},
        {11, 16},{10, 17},{10, 15},{9, 18},{9, 16},{9, 14},{8, 19},{8, 17},{8, 15},{8, 13},
        {11, 4},{10, 5},{10, 3},{9, 6},{9, 4},{9, 2},{8, 7},{8, 5},{8, 3},{8, 1},
        {6, 19},{6, 17},{6, 15},{6, 13},{5, 18},{5, 16},{5, 14},{4, 17},{4, 15},{3, 16},
        {6, 7},{6, 5},{6, 3},{6, 1},{5, 6},{5, 4},{5, 2},{4, 5},{4, 3},{3, 4},
        {5, 10},{4, 11},{4, 9},{3, 12},{3, 10},{3, 8},{2, 13},{2, 11},{2, 9},{2, 7},
        {5, 22},{4, 23},{4, 21},{3, 24},{3, 22},{3, 20},{2, 25},{2, 23},{2, 21},{2, 19}};

    };

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class skinsampleDraw : public vDraw {
private:
std::list<std::tuple<int, int>> t_map{};
public:
    void initialise(){
        Xlimit = 600;
        Ylimit =600;    
        
        t_map={{24, 7},{24, 9},{24, 11},{24, 13},{23, 12},{23, 10},{23, 8},{22, 11},{22, 9},{21, 10},
        {24, 19},{24, 21},{24, 23},{24, 25},{23, 24},{23, 22},{23, 20},{22, 23},{22, 21},{21, 22},
        {23, 16},{22, 17},{22, 15},{21, 18},{21, 16},{21, 14},{20, 19},{20, 17},{20, 15},{20, 13},
        {23, 4},{22, 5},{22, 3},{21, 4},{21, 6},{21, 2},{20, 7},{20, 5},{20, 3},{20, 1},
        {18, 19},{18, 17},{18, 15},{18, 13},{17, 18},{17, 16},{17, 14},{16, 17},{16, 15},{15, 16},
        {18, 7},{18, 5},{18, 3},{18, 1},{17, 6},{17, 4},{17, 2},{16, 5},{16, 3},{15, 4},
        {17, 10},{16, 11},{16, 9},{15, 12},{15, 10},{15, 8},{14, 13},{14, 11},{14, 9},{14, 7},
        {17, 22},{16, 23},{16, 21},{15, 24},{15, 22},{15, 20},{14, 25},{14, 23},{14, 21},{14, 19},
        {12, 19},{12, 21},{12, 23},{12, 25},{11, 24},{11, 22},{11, 20},{10, 23},{10, 21},{9, 22},
        {12, 13},{12, 11},{12, 9},{12, 7},{11, 12},{11, 10},{11, 8},{10, 11},{10, 9},{9, 10},
        {11, 16},{10, 17},{10, 15},{9, 18},{9, 16},{9, 14},{8, 19},{8, 17},{8, 15},{8, 13},
        {11, 4},{10, 5},{10, 3},{9, 6},{9, 4},{9, 2},{8, 7},{8, 5},{8, 3},{8, 1},
        {6, 19},{6, 17},{6, 15},{6, 13},{5, 18},{5, 16},{5, 14},{4, 17},{4, 15},{3, 16},
        {6, 7},{6, 5},{6, 3},{6, 1},{5, 6},{5, 4},{5, 2},{4, 5},{4, 3},{3, 4},
        {5, 10},{4, 11},{4, 9},{3, 12},{3, 10},{3, 8},{2, 13},{2, 11},{2, 9},{2, 7},
        {5, 22},{4, 23},{4, 21},{3, 24},{3, 22},{3, 20},{2, 25},{2, 23},{2, 21},{2, 19}};

    };
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

class circleDraw : public vDraw {

protected:

    std::map<int, ev::event<ev::GaussianAE>> persistance;
    int stagnantCount;

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class blobDraw : public vDraw {

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

class isoInterestDraw : public isoDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class isoCircDraw : public isoDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class overlayStereoDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class saeDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

#endif


