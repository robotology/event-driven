#include "blobTracker.h"


BlobTracker::BlobTracker(double x0, double y0, double sig_x2, double sig_y2, double sig_xy, 
			 double alpha_pos, double alpha_shape, double k, double up_thresh, double down_thresh, double delete_thresh){
  cen_x_ = x0;
  cen_y_ = y0;

  sig_x2_ = sig_x2;
  sig_y2_ = sig_y2;
  sig_xy_ = sig_xy;

  alpha_pos_ = alpha_pos;
  alpha_shape_ = alpha_shape;

  fixed_shape_ = false;

  for(int ii=0; ii<360; ii++){
    theta_.push_back(ii*M_PI/180);
  }

  k_ = k;

  up_thresh_ = up_thresh;
  down_thresh_ = down_thresh;
  delete_thresh_ = delete_thresh;
  state_ = Inactive;
  activity_ = 0;

  vx_ = 0;
  vy_ = 0;
  ts_last_update_ = 0;
}



BlobTracker::BlobTracker(){
}




BlobTracker::~BlobTracker() {
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



bool BlobTracker::update_activity(double temp_decay){
  activity_*=temp_decay;
  
  // In this stage, the activity can only go down, so it can only go from active to inactive
  if(activity_ < down_thresh_){
    state_ = Inactive;
  }

  // If it drops below the delete threshold, we return true
  if(activity_ < delete_thresh_){
    return true;
  }
  else{
    return false;
  }
}



double BlobTracker::dist2event(int ev_x, int ev_y) {
  double dx = ev_x - cen_x_;
  double dy = ev_y - cen_y_;

  double dist = sqrt(dx*dx+dy*dy); //compute Euclidean distance

  return dist;

}


void BlobTracker::update_position(int ev_x, int ev_y, double p, int ts) {
  activity_ += 1;//p;
  if(state_==Inactive && activity_>up_thresh_){
    state_=Active;
  }

  double delta_sig_x2 = (ev_x-cen_x_)*(ev_x-cen_x_);
  double delta_sig_y2 = (ev_y-cen_y_)*(ev_y-cen_y_);
  double delta_sig_xy;

  if(fixed_shape_){
    delta_sig_x2 = (delta_sig_x2 + delta_sig_y2)/2;
    delta_sig_y2 = delta_sig_x2;
    delta_sig_xy = 0;
  }
  else{
    delta_sig_xy = (ev_x-cen_x_)*(ev_y-cen_y_);
  }

  sig_x2_ = (1-alpha_shape_)*sig_x2_ + alpha_shape_*delta_sig_x2;
  sig_y2_ = (1-alpha_shape_)*sig_y2_ + alpha_shape_*delta_sig_y2;
  sig_xy_ = (1-alpha_shape_)*sig_xy_ + alpha_shape_*delta_sig_xy;

  double old_x = cen_x_;
  double old_y = cen_y_;
  
  cen_x_ = (1-alpha_pos_)*cen_x_ + alpha_pos_*ev_x;
  cen_y_ = (1-alpha_pos_)*cen_y_ + alpha_pos_*ev_y;

  double alpha = 0.2;
  double dt = ts-ts_last_update_;

  if(dt>0){
    vx_ = (1-alpha)*vx_ + alpha*(cen_x_-old_x)/dt;
    vy_ = (1-alpha)*vy_ + alpha*(cen_y_-old_y)/dt;
  }
  ts_last_update_ = ts;
}



void BlobTracker::displace(double dx, double dy){
  cen_x_ += dx;
  cen_y_ += dy;
}


void BlobTracker::fix_shape(bool fix){
  fixed_shape_ = fix;
}

void BlobTracker::move_to(double x, double y){
  cen_x_ = x;
  cen_y_ = y;
}



void BlobTracker::get_ellipse_parameters(double &a, double &b, double &alpha){
  // Compute the eigenvalues of the covariance matrix
  double tmp = sqrt( (sig_x2_ - sig_y2_) * (sig_x2_ - sig_y2_) + 4*sig_xy_*sig_xy_ );
  double l_max = 0.5*(sig_x2_ + sig_y2_ + tmp);
  double l_min = 0.5*(sig_x2_ + sig_y2_ - tmp);

  a = sqrt(l_max);  
  b = sqrt(l_min);
  alpha = 0.5*atan2f(2*sig_xy_, sig_y2_ - sig_x2_);
}

void BlobTracker::get_gauss_parameters(double &sig_x2, double &sig_y2, double &sig_xy){
  sig_x2 = sig_x2_;
  sig_y2 = sig_y2_;
  sig_xy = sig_xy_;  
}


void BlobTracker::get_center(double &cen_x, double &cen_y){
  cen_x = cen_x_;
  cen_y = cen_y_;
}


// Inactive trackers are displayed in blue, active trackers in red
void BlobTracker::display(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img, yarp::sig::PixelRgb color){

  double a, b, alpha;
  this->get_ellipse_parameters(a, b, alpha);

  a*=k_;
  b*=k_;
	
  alpha +=M_PI/2;
  alpha *=-1;

  double c_a = cos(alpha);
  double s_a = sin(alpha);
	
  for(int ii=0; ii<theta_.size(); ii++){
    double x0 = a*cos(theta_[ii]);
    double y0 = b*sin(theta_[ii]);
    
    int x = round(cen_x_ + x0*c_a - y0*s_a);
    int y = round(cen_y_ + x0*s_a + y0*c_a);
    if(x>0 && x<128 && y>0 && y<128){
      img.pixel(y, x) = color;
    }
  }
}



void BlobTracker::reset(double x0, double y0, double sig_x2, double sig_y2, double sig_xy){
  cen_x_ = x0;
  cen_y_ = y0;

  sig_x2_ = sig_x2;
  sig_y2_ = sig_y2;
  sig_xy_ = sig_xy;
}	



void BlobTracker::get_norm_v(double &vx, double &vy){
  double v = sqrt(vx_*vx_ + vy_*vy_);
  vx = vx_/v;
  vy = vy_/v;
}
