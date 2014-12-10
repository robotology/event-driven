#include "tracker_pool.h"


TrackerPool::TrackerPool(double sig_x, double sig_y, double sig_xy, double alpha_pos, double alpha_shape, double k, 
			 double max_dist, bool fixed_shape, double tau_act, double up_thresh, double down_thresh, double delete_thresh,
			 double alpha_rep, int d_rep, int max_nb_trackers, int nb_ev_regulate){

  std::string filename = "distances.txt";
  output_file_ = fopen (filename.c_str(), "w");
  if (output_file_ == NULL) 
    perror ("Error opening file");

  max_dist_ = max_dist;
  tau_act_ = tau_act;

  alpha_rep_ = alpha_rep;
  d_rep_ = d_rep;

  dist_thresh_ = 30;
  vel_thresh_ = 40;
  acc_thresh_ = 400;

  sig_x2_ = sig_x*sig_x;
  sig_y2_ = sig_y*sig_y;
  sig_xy_ = sig_xy;
  for(int ii=0; ii<max_nb_trackers; ii++){
    trackers_.push_back(BlobTracker(0, 0, sig_x2_, sig_y2_, sig_xy_, alpha_pos, alpha_shape, k, up_thresh, down_thresh, delete_thresh));
    trackers_[ii].fix_shape(fixed_shape);
    to_reset_.push_back(ii);
    std::vector<bool> temp_bool;
    std::vector<double> temp_double;
    for(int jj=0; jj<max_nb_trackers; jj++){
      temp_bool.push_back(false);
      temp_double.push_back(0);
    }
    dist_.push_back(temp_double);
    vel_.push_back(temp_double);
    acc_.push_back(temp_double);
    clapping_.push_back(temp_bool);
  }

  span_ = 5;
  for(int ii=0; ii<span_; ii++){
    last_dist_.push_back(0);
    last_vel_.push_back(0);
    last_acc_.push_back(0);
  }
  max_nb_tr_ = max_nb_trackers;

  nb_ev_regulate_ = nb_ev_regulate;
  count_ = 0;
}



TrackerPool::~TrackerPool(){
  fclose(output_file_);
}



void TrackerPool::update(int ev_x, int ev_y, int ev_t){
  
  // If this is the first event that we receive, we set its timestamp as the first one for the regulation process
  if(count_ == 0){
    ts_last_reg_ = ev_t;
  }

  double max_p = 0;
  double max_ind = -1;

  // We look for the tracker with the biggest p
  for(int ii=0; ii<trackers_.size(); ii++){
    if(trackers_[ii].dist2event(ev_x, ev_y) < max_dist_){
      double p = trackers_[ii].compute_p(ev_x, ev_y);
      if(p>max_p || max_ind ==-1){
	max_p = p; 
	max_ind = ii;
      }
    }
  }

  // If there was not any tracker close enough, we take one of the filters to reset to te position of the event
  if(max_ind == -1){
    if(to_reset_.size()>0){
      trackers_[to_reset_.back()].reset(ev_x, ev_y, sig_x2_, sig_y2_, sig_xy_);
      to_reset_.pop_back();
    }
  }
  // Otherwise, we update the one with the biggest p
  else{
    trackers_[max_ind].update_position(ev_x, ev_y, max_p, ev_t);
  }

  count_++;
  if(count_%nb_ev_regulate_==0){
    regulate_pool(ev_t);
  }
}



void TrackerPool::get_tracker_center(std::vector<double> &cen_x, std::vector<double> &cen_y){
  double temp_x, temp_y;
  cen_x.clear();
  cen_y.clear();

  for(int ii=0; ii<trackers_.size(); ii++){
    trackers_[ii].get_center(temp_x, temp_y);
    cen_x.push_back(temp_x);
    cen_y.push_back(temp_y);
  }  
}



void TrackerPool::get_collisions(std::vector<double> &x_coll, std::vector<double> &y_coll){
  for(int ii=0; ii<x_collision_.size(); ii++){
    printf("[TrackerPool] x collision %f\n", x_collision_[ii]);
  }
  x_coll = x_collision_;
  y_coll = y_collision_;
  x_collision_.clear();
  y_collision_.clear();
}



void TrackerPool::get_ellipse_parameters(std::vector<double> &a, std::vector<double> &b, std::vector<double> &alpha){
  a.clear();
  b.clear();
  alpha.clear();
  double temp_a, temp_b, temp_alpha;

  for(int ii=0; ii<trackers_.size(); ii++){
    trackers_[ii].get_ellipse_parameters(temp_a, temp_b, temp_alpha);
    a.push_back(temp_a);
    b.push_back(temp_b);
    alpha.push_back(temp_alpha);
  }  
}

void TrackerPool::display(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img){
  for(int ii=0; ii<trackers_.size(); ii++){
    if(trackers_[ii].is_active()){
      //yarp::sig::PixelRgb color = trackers_[ii].is_active()?yarp::sig::PixelRgb(0, 0, 255):yarp::sig::PixelRgb(0, 255, 0);
      yarp::sig::PixelRgb color = yarp::sig::PixelRgb(0, 0, 255);
      trackers_[ii].display(img, color);
    }
  }

  std::vector<int> to_delete;
  for(int ii=0; ii<collisions_disp_.size(); ii++){
    for(int jj=-1; jj<1; jj++){
      for(int kk=-1; kk<1; kk++){
        int x = collisions_disp_[ii].x + jj;
        int y = collisions_disp_[ii].y + kk;
        if(x>=0 && x<128 && y>=0 && y<128){
	  img.pixel(y, x) = yarp::sig::PixelRgb(20, 0, 255);
        }
      }
    }
    if(++collisions_disp_[ii].count_disp == 100){
      to_delete.push_back(ii);
    }
  }

  while(to_delete.size()>0){
    int to_del = to_delete.back();
    collisions_disp_.erase(collisions_disp_.begin() + to_del);
    to_delete.pop_back();
  }
}



void TrackerPool::set_collision_det_param(double dist_thresh, double vel_thresh, double acc_thresh){
  dist_thresh_ = dist_thresh;
  vel_thresh_ = vel_thresh;
  acc_thresh_ = acc_thresh;
}



void TrackerPool::get_tracker(BlobTracker &blobT, int index){
  blobT = trackers_[index];
}	

void TrackerPool::get_pool_size(int &index){
  index = trackers_.size();
}


void TrackerPool::regulate_pool(int ts){
  // Apply the exponential decay of activity, and remove the filters that become inactive
  //printf("[tracker pool] : Regulating pool\n");
  int dt = ts-ts_last_reg_;
  // Vector containing the indices of the trackers that should be deleted

  if(dt>0){
    double decay = exp(-(ts-ts_last_reg_)/tau_act_);
    // We update the activity of each tracker.
    for(int ii=0; ii<trackers_.size(); ii++){
      if(trackers_[ii].update_activity(decay)){
	to_reset_.push_back(ii);
      } 
    }
  }
  apply_rep_field();
  // We look for collisions
  look_for_collisions(ts);
  ts_last_reg_ = ts;
}



void TrackerPool::apply_rep_field(){
  // Apply repulsion between filters
  for(int ii=0; ii<trackers_.size(); ii++){
    for(int jj=ii+1; jj<trackers_.size(); jj++){
      // We compute the distance
      int dx = trackers_[ii].get_x() - trackers_[jj].get_x();
      int dy = trackers_[ii].get_y() - trackers_[jj].get_y();
      double dist = sqrt(dx*dx + dy*dy);
      //printf("distance = %f and d_rep = %d\n", dist, d_rep_);
	
      // If the distance is smaller than the minimum, we apply the repulsion filter
      if(dist<d_rep_){
	// The repulsion field will be a liner function of the distance. If the distance is zero, then it is one. If
	// it is equal to d_rep, it will be zero
	//printf("Alpha rep = %f\n", alpha_rep_);
	double delta = alpha_rep_*(1-dist/d_rep_);
        //printf("distance = %f - delta = %f\n", dist, delta);
	int dir_x = dx>0?1:-1;
	int dir_y = dy>0?1:-1;
        double act_sum = trackers_[ii].get_act()+trackers_[jj].get_act();
        double f_i, f_j;

	if(act_sum > 0){
	  f_i = trackers_[jj].get_act()/act_sum;
	  f_j = trackers_[ii].get_act()/act_sum;
	}
	else{
	  f_i = 0.5;
	  f_j = 0.5;
	}
	trackers_[ii].displace(f_i*delta*dir_x, f_i*delta*dir_y);
	trackers_[jj].displace(-f_j*delta*dir_x, -f_j*delta*dir_y);
      }
    }
  }
}



void TrackerPool::look_for_collisions(int ts){
  // Approach based on the relative speed between the filters
  for(int ii=0; ii<trackers_.size(); ii++){
    for(int jj=ii+1; jj<trackers_.size(); jj++){
      if(trackers_[ii].is_active() && trackers_[jj].is_active()){
        int dx = trackers_[ii].get_x() - trackers_[jj].get_x();
        int dy = trackers_[ii].get_y() - trackers_[jj].get_y();

        // As distance we use the average of the last span distances
        last_dist_.pop_front();
        last_dist_.push_back(sqrt(dx*dx + dy*dy));
        double new_dist = 0;
        for (std::list<double>::iterator it=last_dist_.begin(); it != last_dist_.end(); ++it){
          new_dist += *it;
        }
        new_dist/=span_;

        int dt = (ts-ts_last_reg_);
	if(dt>0){
          // New speed
          last_vel_.pop_front();
          last_vel_.push_back(1e6*(new_dist - dist_[ii][jj])/dt);
          double new_vel = 0;
          for (std::list<double>::iterator it=last_vel_.begin(); it != last_vel_.end(); ++it){
            new_vel += *it;
          }
          new_vel/=span_;
          
          // New acceleration
          last_acc_.pop_front();
          last_acc_.push_back(1e6*(new_vel - vel_[ii][jj])/dt);
          double new_acc = 0;
          for (std::list<double>::iterator it=last_acc_.begin(); it != last_acc_.end(); ++it){
            new_acc += *it;
          }
          new_acc/=span_;

          acc_[ii][jj] = new_acc;
	  vel_[ii][jj] = new_vel;
        }
        dist_[ii][jj] = new_dist;

     //   printf("...\n");	
	//printf("Relative speed = %f\n", rel_speed_[ii][jj]);
        if(!clapping_[ii][jj]){
            if(dist_[ii][jj] < dist_thresh_ && vel_[ii][jj] > -vel_thresh_ && vel_[ii][jj] < vel_thresh_ && acc_[ii][jj] > acc_thresh_){
              clapping_[ii][jj] = true;
	      double x_coll = (trackers_[ii].get_x() + trackers_[jj].get_x())/2;
	      double y_coll = (trackers_[ii].get_y() + trackers_[jj].get_y())/2;

	      x_collision_.push_back(x_coll);
              y_collision_.push_back(y_coll);

	      Collision new_collision;
              new_collision.x = x_coll;
              new_collision.y = y_coll;
              new_collision.count_disp = 0;
              collisions_disp_.push_back(new_collision);

              printf("Clap!!\n");
            }
        }
        else if(dist_[ii][jj]>2*dist_thresh_){
          clapping_[ii][jj] = false;
        }
      }
    //fprintf(output_file_, "%f %f %f %d\n", dist_[ii][jj], vel_[ii][jj], acc_[ii][jj], ts);
    }
  }

}
