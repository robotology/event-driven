#ifndef __BLOB_TRACKER_H
#define __BLOB_TRACKER_H

#include <math.h>
#include <yarp/sig/all.h>
#include <vector>

class BlobTracker {
 public:
  enum State{
    Active,
    Inactive,
  };

  BlobTracker(double x0, double y0, double sig_x2, double sig_y2, double sig_xy, double alpha_pos, double alpha_shape, 
	      double k, double up_thresh, double down_thresh, double delete_thresh);
  BlobTracker();	
  ~BlobTracker();

  double compute_p(int ev_x, int ev_y);

  // Applies the given temporal decay. It will return true if the tracker goes from active to inactive (which tipically means that it should be removed)
  bool update_activity(double temp_decay);

  void update_position(int ev_x, int ev_y, double p, int ts);
    
  void displace(double dx, double dy);

  void fix_shape(bool fix);

  void move_to(double x, double y);

  void get_ellipse_parameters(double &a, double &b, double &alpha);

  void get_center(double &cen_x, double &cen_y);

  void display(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img, yarp::sig::PixelRgb color);

  double dist2event(int ev_x, int ev_y);

  void reset(double x0, double y0, double sig_x2, double sig_y2, double sig_xy);

  void get_norm_v(double &vx, double &vy);

  // Accessors to some of the tracker's properties
  inline bool is_active(){return state_==Active;}
  inline int get_x(){return cen_x_;}
  inline int get_y(){return cen_y_;}
  inline double get_vx(){return vx_;}
  inline double get_vy(){return vy_;}
  inline double get_act(){return activity_;}

 protected:
  double cen_x_, cen_y_, sig_x2_, sig_y2_, sig_xy_;
  double vx_, vy_;
  double alpha_pos_, alpha_shape_;
  double up_thresh_, down_thresh_, delete_thresh_;
  bool fixed_shape_;
  double activity_;
  int ts_last_update_;  

  State state_;

  double k_;
  // Vector containing the degrees (in radians) for plotting the ellipse
  std::vector<double> theta_;

};

#endif /* __GAUSSIAN_BLOB_TRACKER_H */
