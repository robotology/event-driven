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
