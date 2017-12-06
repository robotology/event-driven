/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
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

#ifndef __BLOB_TRACKER_H
#define __BLOB_TRACKER_H

#include <cmath>

class BlobTracker {

protected:

    enum State{
        Active,
        Inactive,
        Free,
    };

    State state_;

    double cen_x_, cen_y_;
    double sig_x2_, sig_y2_, sig_xy_;
    double vLastX, vLastY;
    double vx_, vy_;
    double alpha_pos_, alpha_shape_;
    bool fixed_shape_;
    double activity_;
    unsigned long int ts_last_update_;

public:

    BlobTracker();

    //initialisation
    bool initialisePosition(double x, double y);
    bool initialiseShape(double sigX, double sigY2, double sigXY,
                        double alpha_pos, double alpha_shape, bool fix);

    //mathematical comparison
    double dist2event(int ev_x, int ev_y);
    double compute_p(int ev_x, int ev_y);

    //update with new event
    bool addActivity(int x, int y, unsigned long int ts,
                     double Tact, double Tevent);
    bool decayActivity(int dt, double tau, double Tinact, double Tfree);
    void clusterSpiked();

    //updating and getting state
    void isNoLongerFree() { state_ = Inactive; }
    inline bool is_active(){return state_==Active;}
    inline bool is_on(){return state_==Active || state_==Inactive;}
    inline bool isFree() {return state_ == Free;}

    //getting blob position etc.
    inline double get_sigx2() {return sig_x2_;}
    inline double get_sigy2() {return sig_y2_;}
    inline double get_sigxy() {return sig_xy_;}
    inline int get_x(){return cen_x_;}
    inline int get_y(){return cen_y_;}
    inline double get_vx(){return vx_;}
    inline double get_vy(){return vy_;}
    inline double get_act(){return activity_;}



};

#endif /* __GAUSSIAN_BLOB_TRACKER_H */
