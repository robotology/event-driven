/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi and Arren Glover
 * email:  chiara.bartolozzi@iit.it, arren.glover@iit.it
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

#include "blobTracker.h"
#define _USE_MATH_DEFINES
#include <math.h>

BlobTracker::BlobTracker()
{

    cen_x_ = 0;
    cen_y_ = 0;
    vLastX = 0;
    vLastY = 0;

    sig_x2_ = 25;
    sig_y2_ = 25;
    sig_xy_ = 0;

    alpha_pos_ = 0.1;
    alpha_shape_ = 0.01;

    fixed_shape_ = false;

    state_ = Free;

    activity_ = 0;

    vx_ = 0;
    vy_ = 0;
    ts_last_update_ = 0;

}

bool BlobTracker::initialisePosition(double x, double y)
{
    if(state_ != Free)
        return false;
    cen_x_ = x;
    cen_y_ = y;
    return true;
}

bool BlobTracker::initialiseShape(double sigX2, double sigY2, double sigXY,
                                  double alpha_pos, double alpha_shape,
                                  bool fix)
{
    if(state_ != Free)
        return false;
    sig_x2_ = sigX2;
    sig_y2_ = sigY2;
    sig_xy_ = sigXY;
    alpha_pos_ = alpha_pos;
    alpha_shape_ = alpha_shape;
    fixed_shape_ = fix;
    return true;
}



bool BlobTracker::addActivity(int x, int y, unsigned long int ts,
                              double Tact, double Tevent)
{
    activity_ += 1;//p;

    double delta_sig_x2 = (x-cen_x_)*(x-cen_x_);
    double delta_sig_y2 = (y-cen_y_)*(y-cen_y_);
    double delta_sig_xy;

    if(fixed_shape_){
        delta_sig_x2 = (delta_sig_x2 + delta_sig_y2)/2;
        delta_sig_y2 = delta_sig_x2;
        delta_sig_xy = 0;
    }
    else{
        delta_sig_xy = (x-cen_x_)*(y-cen_y_);
    }

    sig_x2_ = (1-alpha_shape_)*sig_x2_ + alpha_shape_*delta_sig_x2;
    sig_y2_ = (1-alpha_shape_)*sig_y2_ + alpha_shape_*delta_sig_y2;
    sig_xy_ = (1-alpha_shape_)*sig_xy_ + alpha_shape_*delta_sig_xy;

    double old_x = cen_x_;
    double old_y = cen_y_;

    cen_x_ = (1-alpha_pos_)*cen_x_ + alpha_pos_*x;
    cen_y_ = (1-alpha_pos_)*cen_y_ + alpha_pos_*y;

    double alpha = 0.2;
    double dt = ts-ts_last_update_;

    if(dt>0){
        vx_ = (1-alpha)*vx_ + alpha*(cen_x_-old_x)/dt;
        vy_ = (1-alpha)*vy_ + alpha*(cen_y_-old_y)/dt;
    }
    ts_last_update_ = ts;

    //return true if just turned on
    if(!is_active() && activity_ > Tact) {
        state_ = Active;
        return true;
    }

    //return true if moved enough
    double distance = sqrt(std::pow(vLastX - cen_x_, 2.0) +
                           std::pow(vLastY - cen_y_, 2.0));
    if(is_active() && distance > Tevent) {
        return true;
    }

    return false;
}

bool BlobTracker::decayActivity(int dt, double tau, double Tinact, double Tfree)
{
    State prev_state = state_;
    activity_ *= exp(-dt/tau);

    if(activity_ < Tfree){
        state_ = Free;
    }
    else if(activity_ < Tinact){
        state_ = Inactive;
    }

    return prev_state != state_;
}

void BlobTracker::clusterSpiked()
{
    vLastX = cen_x_;
    vLastY = cen_y_;
}

double BlobTracker::compute_p(int ev_x, int ev_y) {
    double dx = ev_x - cen_x_;
    double dy = ev_y - cen_y_;

    // We compute the determinant of the covariance matrix
    double det = sig_x2_*sig_y2_-sig_xy_*sig_xy_;

    // That we use for computing its inverse. We directly compute the resulting probability
    double tmp = (1/det)*(dx*dx*sig_y2_-2*dx*dy*sig_xy_+dy*dy*sig_x2_);

    return 1.0/(2*M_PI*sqrt(det))*exp(-0.5*tmp);
}

double BlobTracker::dist2event(int ev_x, int ev_y) {
    double dx = ev_x - cen_x_;
    double dy = ev_y - cen_y_;

    double dist = sqrt(dx*dx+dy*dy); //compute Euclidean distance

    return dist;

}
