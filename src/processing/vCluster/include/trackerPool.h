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

#ifndef __TRACKERPOOL_H
#define __TRACKERPOOL_H

#include "blobTracker.h"

#include <iCub/eventdriven/all.h>
#include <vector>

class TrackerPool {

protected:

    //we will store trackers in a vector
    std::vector<BlobTracker> trackers_;

    int nb_ev_regulate_, count_;
    unsigned long int  ts_last_reg_;
    double decay_tau;
    double Tact, Tinact, Tfree, Tevent;
    double max_dist;
    bool fixed_shape_;
    double sig_x2_, sig_y2_, sig_xy_;
    double alpha_pos, alpha_shape;
    double clusterLimit;

    int getNewTracker();
    ev::event<ev::GaussianAE> makeEvent(int i, int ts);
    ev::vtsHelper unwrap;


    // Parameters of the repulsive field
    //double alpha_rep_;
    //int d_rep_;
    //void apply_rep_field();


public:

    TrackerPool();

    void setInitialParams(double sig_x, double sig_y, double sig_xy,
                          double alpha_pos, double alpha_shape,
                          bool fixed_shape);
    void setDecayParams(double decay_tau, double Tact, double Tinact,
                        double Tfree, double Tevent, int rate);
    void setComparisonParams(double max_dist);
    void setClusterLimit(int limit);

    int update(ev::event<ev::AddressEvent> v,
               std::vector<ev::event<ev::GaussianAE> > &clEvts);

};

#endif /* __GAUSSIAN_BLOB_TRACKER_H */
