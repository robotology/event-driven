#ifndef __TRACKER_POOL_H
#define __TRACKER_POOL_H

#include "blob_tracker.h"

#include <math.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <fstream>

// Class implementing a pool of Gaussian Trackers, that looks for collisions between them.
// The different trackers have a repulsive force that tends to mantain them at a certain distance
class TrackerPool {
 public:
  // TrackerPool(double sig_x, double sig_y, double sig_xy, double alpha_pos, double alpha_shape, double k, 
  //             double max_dist, bool fixed_shape, double tau_act, double up_thresh, double down_thresh, double delete_thresh, 
  //             double alpha_rep, int d_rep, int max_nb_trackers, int nb_ev_regulate);
  //
  // Contructor
  //
  // Parameters:
  //   - sig_x, sig_y, sig_xy: initial values of the gaussian trackers 
  TrackerPool(double sig_x, double sig_y, double sig_xy, double alpha_pos, double alpha_shape, double k, 
	      double max_dist, bool fixed_shape, double tau_act, double up_thresh, double down_thresh, double delete_thresh, 
	      double alpha_rep, int d_rep, int max_nb_trackers, int nb_ev_regulate);

  ~TrackerPool();

  void get_tracker (BlobTracker &, int);

  void update(int ev_x, int ev_y, int ev_t);

  void get_tracker_center(std::vector<double> &cen_x, std::vector<double> &cen_y);

  void get_collisions(std::vector<double> &x_coll, std::vector<double> &y_coll);

  void get_ellipse_parameters(std::vector<double> &a, std::vector<double> &b, std::vector<double> &alpha);

  void display(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img);

  void set_collision_det_param(double dist_thresh, double vel_thresh, double acc_thresh);

 protected:

  struct Collision{
    int x;
    int y;
    int count_disp;
  };

  void regulate_pool(int ts);

  void apply_rep_field();

  void look_for_collisions(int ts);

  std::vector<BlobTracker> trackers_;
  yarp::os::Port image_port;

  // We will regulate the pool every fixed number of events
  int nb_ev_regulate_, count_;
  int ts_last_reg_;

  // Parameters of the repulsive field
  double alpha_rep_;
  int d_rep_;

  std::vector<int> to_reset_;
  int max_nb_tr_;

  // Initial parameters of the blob trackers
  double max_dist_, tau_act_;
  bool fixed_shape_;
  double sig_x2_, sig_y2_, sig_xy_;

  // Variables for the collision detector
  double dist_thresh_, vel_thresh_, acc_thresh_;
  int max_dt_;
  std::vector<std::vector<bool> > clapping_;
  std::vector<std::vector<double> > dist_, vel_, acc_;
  std::list<double> last_dist_, last_vel_, last_acc_;
  int span_;
  std::vector<Collision> collisions_disp_;
  std::vector<double> x_collision_, y_collision_;

  FILE *output_file_;
};

#endif /* __GAUSSIAN_BLOB_TRACKER_H */
