/*******************************************************************************
                                                                               *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)         *
 * All Rights Reserved.                                                        *
                                                                               *
                                                                              */

/**
 * @file: IMUCalibDumper/src/IMUCalibDumper.cpp
 * @authors:    Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
 */

#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>
#include <event-driven/all.h>

// These macros serve to anotate which sensors' data were received.
// If the i^th sensor is received, it adds 1 to the i^th bit of the variable SENSORS
#define SENSORS sensors
#define DEF_SENSORS() unsigned int SENSORS = 0
#define SET_SENSOR(x) SENSORS += (1 << x)
#define ALL_SENSORS 1023 // If the 10 sensors are set, the value is 1023
#define RESET_SENSORS() SENSORS = 0

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace ev;
using namespace yarp::os;

class IMUCalibDumper : public RFModule, public Thread {

private:
    vReadPort< vector<IMUevent>  > imu_port; //!< input port to receive data from the IMU

    std::vector<double> imu_state; //!< saves the IMU values in order

    imuHelper imu_helper;

    ev::vtsHelper vtsHelper; // common for all LOGs
    std::string accFileName, gyrFileName;
    fs::path accFilePath, gyrFilePath;
    std::vector<std::vector<double>> accData;
    std::vector<std::vector<double>> gyrData;

public:
    /*!
     * Open and configure all the resources
     *
     * \param rf contains the command-line options
     *
     * \return bool true/false if success/fail
     */
    bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/IMUCalibDumper")).asString()).c_str());

        //open io ports
        if(!imu_port.open(getName() + "/IMU:i")) {
            yError() << "Could not open input port";
            return false;
        }

        //inilialize the vector which saves the values
        imu_state = std::vector<double>(10, 0);

        accFileName = static_cast<std::string>(rf.check("accFileName", Value(std::string(std::getenv("HOME"))+"/results/IMUDataDump/acc.mat")).asString());
        gyrFileName = static_cast<std::string>(rf.check("gyrFileName", Value(std::string(std::getenv("HOME"))+"/results/IMUDataDump/gyr.mat")).asString());
        accFilePath = accFileName;
        gyrFilePath = gyrFileName;

        //start the asynchronous and synchronous threads
        return Thread::start();
    }

    /*!
     *	Establish the period of the background service thread
     *
     * \return double the period given in seconds
     */
    double getPeriod()
    {
        return 0.5; //period of synchrnous thread
    }

    /*!
     * Stop the thread associated with the IMUCalibDumper module
     *
     * \return bool true/false if success/fail
     */
    bool interruptModule()
    {
        //if the module is asked to stop ask the asynchrnous thread to stop
        return Thread::stop();
    }

    /*!
     * Save the data in the files and close the resources
     */
    void onStop()
    {
        // Create files and save the data
        if(!fs::exists(accFilePath.parent_path()))
        {
            yInfo() << "creating log file folder";
            fs::create_directories(accFilePath.parent_path());
        }
        std::ofstream acclog, gyrlog;
        acclog.open(accFileName, std::ofstream::out | std::ofstream::trunc);
        gyrlog.open(gyrFileName, std::ofstream::out | std::ofstream::trunc);
        acclog << std::scientific;
        gyrlog << std::scientific; 
        // dump acc 
        for(auto vec : accData)
        {
           for(auto entry : vec)
           {
               acclog << "\t" << entry;
           }
           acclog << std::endl;
        }
        acclog.close();

        // dump gyro
        for(auto vec : gyrData)
        {
           for(auto entry : vec)
           {
               gyrlog << "\t" << entry;
           }
           gyrlog << std::endl;
        }
        gyrlog.close();
        
        // clean up
        imu_port.close();
    }

    /*!
     * Background service thread (synchronous)
     *
     * \return bool true/false if success/fail
     */
    bool updateModule()
    {
        // Print data for making easy for the user to follow the calibration pattern
        std::cout << "Acc reding: ";
        if(accData.size() > 0){
            for(auto e : accData.back())
               std::cout << e << ", ";
            std::cout << "\n"; 
        }
        return Thread::isRunning();
    }

    /*!
     * Main functionality of the module. Reads the IMU events stream, gathers the IMU reading from the different sensors and stores them in a single vector for dumping in a file later
     */
    void run()
    {
        Stamp yarpstamp_imu;
        DEF_SENSORS(); // SENSORS keeps track of which sensors are set
        double t0;
        bool firstTime = true;

        while(true) {
            const vector<IMUevent> * q_imu = imu_port.read(yarpstamp_imu);
            if(!q_imu || Thread::isStopping()) return;


            for( ev::IMUevent e_imu : *q_imu)
            {
                SET_SENSOR(e_imu.sensor); // set sensor e_imu.sensor as received
                imu_state[e_imu.sensor] =  e_imu.value;
                    
                // continue receiving data until the 10th reading (index 9)
                if(e_imu.sensor != 9) continue;

                // If the 10 sensors were not received, restarts gathering the data
                bool all_sensors_valid = (SENSORS == ALL_SENSORS);
                RESET_SENSORS();
                if(!all_sensors_valid) continue;

                double currTime = static_cast<double>(vtsHelper(e_imu.stamp))*vtsHelper.tsscaler;
                
                if(firstTime)
                {
                    t0 = currTime;
                    firstTime = false;
                }

                std::vector<double> dacc, dgyr;

                dacc.push_back(currTime-t0);
                dacc.push_back(imu_state[0]);
                dacc.push_back(imu_state[1]);
                dacc.push_back(imu_state[2]);

                dgyr.push_back(currTime-t0);
                dgyr.push_back(imu_state[3]);
                dgyr.push_back(imu_state[4]);
                dgyr.push_back(imu_state[5]);
                
                accData.push_back(dacc);
                gyrData.push_back(dgyr);
                
            }
        }
    }
};

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return -1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "IMUCalibDumper.ini" );
    rf.configure( argc, argv );

    /* create the module */
    IMUCalibDumper instance;
    return instance.runModule(rf);
}

//That empty line
