/*
 * Copyright (C) 2010 iCub Facility
 * Authors: Arren Glover
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __vDraw__
#define __vDraw__

#include <iCub/eventdriven/all.h>
#include <string>
#include <opencv2/opencv.hpp>

/**
 * @brief The vDraw class is the base class from which all vDrawers should
 * inherit. It contains the draw and getTag functions which must be overloaded,
 * and the sensor size for reference.
 */
class vDraw {

protected:

    int Xlimit;
    int Ylimit;
    int stagnantCount;
    int pTS;
    int clearThreshold;
    int twindow;
    bool flip;

    int checkStagnancy(const ev::vQueue &eSet) {
        if(!eSet.size()) return 0;
        if(pTS == eSet.back()->stamp)
            stagnantCount++;
        else
            stagnantCount = 0;
        pTS = eSet.back()->stamp;
        return stagnantCount;
    }

public:

    vDraw() : Xlimit(128), Ylimit(128), stagnantCount(0), pTS(0),
            clearThreshold(30), twindow(781250/2), flip(false) {}
    virtual ~vDraw() {}

    ///
    /// \brief setLimits sets the maximum possible values of the position of
    /// events that could be drawn (mostly governed by the sensor size).
    /// \param Xlimit is the maximum x value (width)
    /// \param Ylimit is the maximium y value (height)
    ///
    void setLimits(int Xlimit, int Ylimit)
    {
        this->Xlimit = Xlimit;
        this->Ylimit = Ylimit;
    }

    void setWindow(int twindow)
    {
        this->twindow = twindow;
    }

    void setFlip(bool flip)
    {
        this->flip = flip;
    }

    virtual void initialise() {}

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

class addressDraw : public vDraw {

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

class lifeDraw : public vDraw {

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
private:

    //angles
    double thetaY;
    double thetaX;
    double CY, SY;
    double CX, SX;

    //image with warped square drawn
    cv::Mat baseimage;

    int tsscalar;
    int Zlimit;
    int imagewidth;
    int imageheight;
    int imagexshift;
    int imageyshift;
    int maxdt;

    //private functions
    void pttr(int &x, int &y, int &z);


public:

    void initialise();

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};


/**
 * @brief createDrawer returns an instance of a drawer that matches the tag
 * specified
 * @param tag is the code of the drawer
 * @return a pointer to a drawer on success. 0 if no appropiate drawer was
 * found
 */
vDraw * createDrawer(std::string tag);




#endif


