#include "trackerPool.h"

TrackerPool::TrackerPool(){}

TrackerPool::TrackerPool(double sig_x, double sig_y, double sig_xy, double alpha_pos, double alpha_shape, double k,
                         double max_dist, bool fixed_shape, double tau_act, double up_thresh, double down_thresh, double delete_thresh,
                         double alpha_rep, int d_rep, int max_nb_trackers, int nb_ev_regulate)
{
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
        //act_.push_back(0);
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



//void TrackerPool::update(int ev_x, int ev_y, int ev_t){
//    
//    // If this is the first event that we receive, we set its timestamp as the first one for the regulation process
//    if(count_ == 0){
//        ts_last_reg_ = ev_t;
//    }
//    
//    double max_p = 0;
//    double max_ind = -1;
//    
//    // We look for the tracker with the biggest p
//    for(int ii=0; ii<trackers_.size(); ii++){
//        if(trackers_[ii].dist2event(ev_x, ev_y) < max_dist_){
//            double p = trackers_[ii].compute_p(ev_x, ev_y);
//            if(p>max_p || max_ind ==-1){
//                max_p = p;
//                max_ind = ii;
//            }
//        }
//    }
//    
//    // If there was not any tracker close enough, we take one of the filters to reset to te position of the event
//    if(max_ind == -1){
//        if(to_reset_.size()>0){
//            trackers_[to_reset_.back()].reset(ev_x, ev_y, sig_x2_, sig_y2_, sig_xy_);
//            to_reset_.pop_back();
//        }
//    }
//    // Otherwise, we update the one with the biggest p
//    else{
//        double act;
//        getMeanActivity(act);
//        trackers_[max_ind].update_position(ev_x, ev_y, max_p, ev_t, act);
//        // add to the single address event the field with the cluster ID it belongs to
//        
//    }
//    
//    count_++;
//    if(count_%nb_ev_regulate_==0){
//        regulate_pool(ev_t);
//    }
//}

int TrackerPool::update(emorph::AddressEvent *ptr, std::vector<emorph::ClusterEventGauss> &clEvts){

    int ev_t = ptr->getStamp();
    int ev_x = ptr->getX();
    int ev_y = ptr->getY();
    
    double max_p = 0;
    //int max_ind = -1;
    int trackId = -1;
    double temp_x, temp_y, temp_xy;
    double deltaPos = 0;
    bool just_activated = false;
    
    
    // If this is the first event that we receive, we set its timestamp as the first one for the regulation process
    if(count_ == 0){
        ts_last_reg_ = ev_t;
    }
    
    // We look for the tracker with the biggest p
    for(int ii=0; ii<trackers_.size(); ii++){
        // only among the Active and Inactive clusters
        if(trackers_[ii].is_on() && trackers_[ii].dist2event(ev_x, ev_y) < max_dist_){
            double p = trackers_[ii].compute_p(ev_x, ev_y);
            if(p>max_p || trackId ==-1){
                max_p = p;
                //max_ind = ii;
                trackId = ii;
            }
        }
    }
    
    // If there was not any tracker close enough, we take one of the Free trackers (to_reset_) and reset it to the position of the event
    if(trackId == -1){
        if(to_reset_.size()>0){
            trackId = to_reset_.back();
            trackers_[trackId].reset(ev_x, ev_y, sig_x2_, sig_y2_, sig_xy_);
            to_reset_.pop_back();
            just_activated = true;
            //fprintf(stdout, "new tracker ID: %d \t to reset size %lu\n",trackId, to_reset_.size());
        } else {
            return trackId;
        }
    }
    // Otherwise, we update the one with the biggest p
    else{
        double act;
        getMeanActivity(act);
        bool wasnot_active = !trackers_[trackId].is_active();
        deltaPos = trackers_[trackId].update_position(ev_x, ev_y, max_p, ev_t, act);
        just_activated = wasnot_active && trackers_[trackId].is_active();
        //fprintf(stdout, "update position ID: %d\n", max_ind);
    }
    
    count_++;
    //regulate the pool only each nb_ev_regulate_ events
    if(count_%nb_ev_regulate_==0){
        regulate_pool(ev_t, clEvts);
        count_ = 1;
    }


    
    // if the tracker is active return the cluster event
    if (trackers_[trackId].is_active() && (just_activated || deltaPos > 5)) {
        trackers_[trackId].clusterSpiked();

        emorph::ClusterEventGauss clep;
        clep.setStamp(ev_t);
        clep.setPolarity(1);
        // center coordinates of the cluster
        trackers_[trackId].get_center(temp_x, temp_y);
        clep.setXCog((int)temp_x);
        clep.setYCog((int)temp_y);
        //get the gauss parameters of each cluster
        trackers_[trackId].get_gauss_parameters(temp_x, temp_y, temp_xy);
        //get the velocity of each cluster
        clep.setXSigma2((int)temp_x);
        clep.setYSigma2((int)temp_y);
        clep.setXYSigma((int)temp_xy);

        trackers_[trackId].getVelocity(temp_x, temp_y);
        clep.setXVel((int)temp_x);
        clep.setYVel((int)temp_y);

        // get the activity of each cluster
        trackers_[trackId].getActivity(temp_x);
        clep.setNumAE(temp_x);
        clep.setID(trackId);
        clEvts.push_back(clep);


    }

    return trackId;
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

//void TrackerPool::getTrackers(std::vector<double> &cen_x, std::vector<double> &cen_y, std::vector<double> &v_x, std::vector<double> &v_y, std::vector<double> &sig_x2, std::vector<double> &sig_y2, std::vector<double> &sig_xy, std::vector<double> &activity, std::vector<bool> &is_active,std::vector<double> &id){
//
//    double temp_x, temp_y, temp_xy;
//    cen_x.clear();
//    cen_y.clear();
//    v_x.clear();
//    v_y.clear();
//    sig_x2.clear();
//    sig_y2.clear();
//    sig_xy.clear();
//    activity.clear();
//    is_active.clear();
//    id.clear();
//    
//    for(int ii=0; ii<trackers_.size(); ii++){
//        // get the center of each cluster
//        trackers_[ii].get_center(temp_x, temp_y);
//        cen_x.push_back(temp_x);
//        cen_y.push_back(temp_y);
//        //get the gauss parameters of each cluster
//        trackers_[ii].get_gauss_parameters(temp_x, temp_y, temp_xy);
//        sig_x2.push_back(temp_x);
//        sig_y2.push_back(temp_y);
//        sig_xy.push_back(temp_xy);
//        //get the velocity of each cluster
//        trackers_[ii].getVelocity(temp_x, temp_y);
//        v_x.push_back(temp_x);
//        v_y.push_back(temp_y);
//        // get the activity of each cluster
//        trackers_[ii].getActivity(temp_x);
//        activity.push_back(temp_x);
//
//        temp_x = trackers_[ii].is_active();
//        is_active.push_back(temp_x);
//        
//        id.push_back(ii);
//    }
//    
//}

void TrackerPool::get_collisions(std::vector<double> &x_coll, std::vector<double> &y_coll){
    for(int ii=0; ii<x_collision_.size(); ii++){
        printf("[TrackerPool] x collision %f\n", x_collision_[ii]);
    }
    x_coll = x_collision_;
    y_coll = y_collision_;
    x_collision_.clear();
    y_collision_.clear();
}

std::vector<TrackerPool::Collision> TrackerPool::get_collisions(){
    return collisions_disp_;
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
            yarp::sig::PixelRgb color = yarp::sig::PixelRgb(0, 0, 255);
            trackers_[ii].display(img, color, ii);
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

//void TrackerPool::get_pool_size(int &index){
//    index = trackers_.size();
//}

int TrackerPool::getPoolSize(){
    return max_nb_tr_-to_reset_.size();
}

void TrackerPool::getMeanActivity(double &act){
    act = 0;

    // mean activity of active and inactive clusters (not free trackers)
    // check how it changes with only active clusters
    
    for(int ii=0; ii<trackers_.size(); ii++){
        if(trackers_[ii].is_active()){
            act += trackers_[ii].get_act();
        }
    }
    act = act/(max_nb_tr_ - getPoolSize());

}


void TrackerPool::regulate_pool(int ts,
                                std::vector<emorph::ClusterEventGauss> &clEvts)
{
    // Apply the exponential decay of activity, and remove the filters that become inactive
    //fprintf(stdout,"[tracker pool] : Regulating pool\n");
    int dt = ts-ts_last_reg_;

    if(dt>0){
        double act;
        getMeanActivity(act);
        double decay = exp(-dt/tau_act_);
        for(int ii=0; ii<max_nb_tr_; ii++){
             // We update the activity of each Active tracker
            if(trackers_[ii].is_on()){
                //fprintf(stdout, "active track ID: %d \n", ii);

                //check to see if the activity changes
                if(trackers_[ii].update_activity(decay,act)){

                    //if it has changed to off it needs to be reset
                    if(!trackers_[ii].is_on())
                        to_reset_.push_back(ii);

                    //otherwise it is just inactive but in either case we
                    //need to produce an event
                    double temp_x, temp_y, temp_xy;                   
                    emorph::ClusterEventGauss clep;
                    clep.setStamp(ts);
                    clep.setPolarity(0);
                    //std::cout << ts << "(" << clep.getStamp() << ") ";
                    //std::cout << 0 << "(" << clep.getPolarity() << ") ";
                    // center coordinates of the cluster
                    trackers_[ii].get_center(temp_x, temp_y);
                    clep.setXCog((int)temp_x);
                    clep.setYCog((int)temp_y);
                    //std::cout << temp_x << "(" << clep.getXCog() << ") ";
                    //std::cout << temp_y << "(" << clep.getYCog() << ") ";
                    //get the gauss parameters of each cluster
                    trackers_[ii].get_gauss_parameters(temp_x, temp_y, temp_xy);
                    //get the velocity of each cluster
                    clep.setXSigma2((int)temp_x);
                    clep.setYSigma2((int)temp_y);
                    clep.setXYSigma((int)temp_xy);
                    //std::cout << temp_x << "(" << clep.getXSigma2() << ") ";
                    //std::cout << temp_y << "(" << clep.getYSigma2() << ") ";
                    //std::cout << temp_xy << "(" << clep.getXYSigma() << ") ";

                    trackers_[ii].getVelocity(temp_x, temp_y);
                    clep.setXVel((int)temp_x);
                    clep.setYVel((int)temp_y);
                    //std::cout << temp_x << "(" << clep.getXVel() << ") ";
                    //std::cout << temp_y << "(" << clep.getYVel() << ") ";

                    // get the activity of each cluster
                    trackers_[ii].getActivity(temp_x);
                    clep.setNumAE(temp_x);
                    clep.setID(ii);
                    //std::cout << temp_x << "(" << clep.getNumAE() << ") ";
                    //std::cout << ii << "(" << clep.getID() << ") ";
                    //std::cout << std::endl;
                    clEvts.push_back(clep);

                    //fprintf(stdout, "reset ID: %d \t to reset size: %lu \n", to_reset_[ii], to_reset_.size());
                    if (to_reset_.size()>max_nb_tr_) {
                        fprintf(stdout, "MERD! il memory leak.... \n\n\n\n");
                    }
                }
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
    for(int ii=0; ii<max_nb_tr_; ii++){
        for(int jj=ii+1; jj<max_nb_tr_; jj++){
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

    for(int ii=0; ii<max_nb_tr_; ii++){
        for(int jj=ii+1; jj<max_nb_tr_; jj++){
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
                        new_collision.timestamp = ts; // H.A.: adding timestamp info for collisions
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
