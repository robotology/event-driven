/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email: valentina.vasco@iit.it
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

#include "vEgoMotion.h"

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace emorph;

/**********************************************************/
bool vEgoMotionModule::configure(ResourceFinder &rf)
{
    //set the name of the module
    string moduleName = rf.check("name", Value("vEgoMotion"),
                                      "module name (string)").asString();
    setName(moduleName.c_str());

    //open and attach the rpc port
    string rpcPortName  = "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        cerr << getName() << " : Unable to open rpc port at " <<
                     rpcPortName << endl;
        return false;
    }

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcPort);


    //set other variables we need from the
    string fileName = rf.check("variable",
                        Value("variable_defualt"),
                        "variable description").asString();

    //set parameters
    double threshold = rf.find("threshold").asDouble();
    if(threshold == 0) threshold = 1;

    //create the thread and pass pointers to the module parameters
    vBotManager = new vBottleManager(moduleName, threshold);
    vBotManager->open();

    return true ;
}

/**********************************************************/
bool vEgoMotionModule::interruptModule()
{
    rpcPort.interrupt();
    vBotManager->interrupt();
    return true;
}

/**********************************************************/
bool vEgoMotionModule::close()
{
    rpcPort.close();
    vBotManager->close();
    delete vBotManager;
    return true;
}

/**********************************************************/
bool vEgoMotionModule::updateModule()
{
    return true;
}

/**********************************************************/
double vEgoMotionModule::getPeriod()
{
    return 0.1;
}

bool vEgoMotionModule::respond(const Bottle &command,
                              Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}


/**********************************************************/
vBottleManager::vBottleManager(const string &_moduleName, double &_threshold)
{
    this->moduleName = _moduleName;

    threshold = _threshold;
    first = true;
    saveFile.open("prediction_vx.txt", ios::out);
    //last_joint_pos[6] = {0, 0, 0, 0, 0, 0};
    
}
/**********************************************************/
bool vBottleManager::open()
{
    //and open the input port
    this->useCallback();

    string inPortName = "/" + moduleName + "/vBottle:i";
    BufferedPort<vBottle>::open(inPortName);

    string inPortEncodersName = "/" + moduleName + "/encoders:i";
    inPortEncoders.open(inPortEncodersName);

    string outPortName = "/" + moduleName + "/vBottle:o";
    outPort.open(outPortName);

    return true;
}

/**********************************************************/
void vBottleManager::close()
{
    //close ports
    this->close();
    inPortEncoders.close();
    outPort.close();

    //remember to also deallocate any memory allocated by this class

}

/**********************************************************/
void vBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    this->interrupt();
    inPortEncoders.interrupt();
    outPort.interrupt();

}

/**********************************************************/
void vBottleManager::onRead(vBottle &vbot)
{
    //create event queue
    vQueue q;
    //create queue iterator
    vQueue::iterator qi;

    //get the current bottle from the encoders
    Bottle *bot = inPortEncoders.read();
    int size = bot->size();

    //get the current timestamp from the encoders
    Stamp t;
    inPortEncoders.getEnvelope(t);
    double curr_ts = t.getTime();
    double last_ts;

    svm_node test_data[size];

    Mat invcovar;

    double joint_pos[size];
    double joint_vel[size];
    //double last_joint_pos[size];

    OpticalFlowEvent outEvent;

    /*prepare output vBottle with address events extended with optical flow events*/
    vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    outBottle.append(vbot);

    //get the event queue in the vBottle bot
    vbot.getAll(q);

    for (qi = q.begin(); qi != q.end(); qi++)
    {
        //discard the first incoming bottle
        if(first == true)
        {
            first = false;
            break;
        }

        for(int b = 0; b < size; b++)
        {
            //get the current position from the encoders
            joint_pos[b] = bot->get(b).asDouble();

            double delta_t = curr_ts - last_ts;

            //compute the current joint velocity
            joint_vel[b] = compute_and_scale_vel(last_joint_pos[b],joint_pos[b], delta_t);
            //cout << "joint_vel (" << b << ") " << joint_vel[b] << endl;

            //update last position with the current position
            last_joint_pos[b] = joint_pos[b];

            //convert to libsvm format
            test_data[b].index = b + 1;
            test_data[b].value = joint_vel[b];
        }

        test_data[size].index = -1;
        test_data[size].value = 0;

        //predict mean and covariance
        vector<double> mean = predict_mean(test_data);
        Mat covar = predict_cov(test_data);
        //cout << mean << endl;
        cout << covar << endl;

        //invert covariance matrix
        invert(covar, invcovar, DECOMP_SVD);

        //get the current optical flow event
        OpticalFlowEvent *ofp = (*qi)->getAs<OpticalFlowEvent>();
        if(ofp)
        {
            x = ofp->getX();
            y = ofp->getY();
            vx = ofp->getVx();    //velocity in s/pixel
            vy = ofp->getVy();    //velocity in s/pixel
            ts = ofp->getStamp();

            line2save.str("");
            line2save << vx << " " << vy << " " << mean[0] << " " << mean[1] << endl;
            saveFile.write( line2save.str().c_str(), line2save.str().size() );

            vector<double> v(2);
            v[0] = vx;
            v[1] = vy;

            //compute Mahalanobis distance
            double mahal_distance = Mahalanobis(v, mean, invcovar);
            //cout << mahal_distance << endl;

            //add the optical flow event to the output bottle only if
            //Mahalanobis distance aboves threshold and hence the flow
            //vector is subject to independent motion
            if (mahal_distance > threshold)
            {
                outEvent.setX(x);
                outEvent.setY(y);
                outEvent.setVx(vx);
                outEvent.setVy(vy);
                outEvent.setStamp(ts);
                outBottle.addEvent(outEvent);

                //send on the processed events
                outPort.write();
            }
        }
    }

    last_ts = curr_ts;

}

/**********************************************************/
//to compute velocity from joint positions and scale by FOV
double vBottleManager::compute_and_scale_vel(double last_pos, double curr_pos, double dt)
{
    double vel;
    double scaling_factor;
    int nr_pixels = 128;
    double fov = 75.63;

    scaling_factor = nr_pixels/fov;
    vel = scaling_factor*((curr_pos - last_pos)/dt);

    if(vel == 0)
        return vel;
    else
        return (1/vel);

}

/**********************************************************/
//to predict the mean
vector<double> vBottleManager::predict_mean(svm_node *test)
{
    //predictions
    double predicted_result_mu_vx = 0;
    double predicted_result_mu_vy = 0;

    vector<double> mu(2);

    //create svm model trained before
    svm_model *mu_vx;
    svm_model *mu_vy;
    //vector<svm_model> a(10);
    //a.size();

    //load the trained model
    mu_vx = svm_load_model("trained_mu_vx.model");
    mu_vy = svm_load_model("trained_mu_vy.model");

    //predict the statistics of optical flow using the trained model
    predicted_result_mu_vx = svm_predict(mu_vx, test);
    predicted_result_mu_vy = svm_predict(mu_vy, test);

    mu[0] = predicted_result_mu_vx;
    mu[1] = predicted_result_mu_vy;

    return mu;

}

/**********************************************************/
//to predict the covariance matrix
Mat vBottleManager::predict_cov(svm_node *test)
{

    //predictions
    double predicted_result_sigma_vx = 0;
    double predicted_result_sigma_vy = 0;
    double predicted_result_sigma_vxvy = 0;

    //create svm model trained before
    svm_model *sigma_vx;
    svm_model *sigma_vy;
    svm_model *sigma_vxvy;

    //load the trained model
    sigma_vx = svm_load_model("trained_sigma_vx.model");
    sigma_vy = svm_load_model("trained_sigma_vy.model");
    sigma_vxvy = svm_load_model("trained_sigma_vxvy.model");

    //predict the statistics of optical flow using the trained model
    predicted_result_sigma_vx = svm_predict(sigma_vx, test);
    predicted_result_sigma_vy = svm_predict(sigma_vy, test);
    predicted_result_sigma_vxvy = svm_predict(sigma_vxvy, test);

    Mat cov = (Mat_<double>(2,2) << predicted_result_sigma_vx,
                 predicted_result_sigma_vxvy, predicted_result_sigma_vxvy,
                 predicted_result_sigma_vy);

    return cov;

}

//empty line to make gcc happy
