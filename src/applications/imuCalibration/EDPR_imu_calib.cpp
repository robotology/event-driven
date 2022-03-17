#include <iostream>
#include <fstream>
#include <iomanip>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

int main(int argc, char** argv)
{
  if( argc < 4 ) {
    std::cout << "Calibrate IMU in the EDPR environment" << std::endl;
    std::cout << "Usage: EDPR_imu_calib <acc file> <gyro file> <output file>" << std::endl;
    return -1;
  }

  vector< TriadData > acc_data, gyro_data;
  
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;  
  importAsciiData( argv[1], acc_data, imu_tk::TIMESTAMP_UNIT_SEC );
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[2]<<endl;  
  importAsciiData( argv[2], gyro_data, imu_tk::TIMESTAMP_UNIT_SEC  );
  ofstream cw(argv[3], ios_base::out|ios_base::trunc);
  if(!cw.is_open()) {
    std::cout << "Error: could not open output .ini file for writing" << std::endl;
    return -1;
  }
  cout<<"Saving Calibration to the file : "<< argv[3] << endl; 
  
  
  CalibratedTriad init_acc_calib, init_gyro_calib;
  init_acc_calib.setBias( Vector3d(0.0, 0.0, 0.0) );
  init_acc_calib.setScale( Vector3d(0.00059855, 0.00059855, 0.00059855) );
  init_gyro_calib.setBias( Vector3d(0.0, 0.0, 0.0) );
  init_gyro_calib.setScale( Vector3d(0.000010653,0.000010653,0.000010653) );
  
  MultiPosCalibration mp_calib;
    
  //mp_calib.setInitStaticIntervalDuration(50.0);
  mp_calib.setInitAccCalibration( init_acc_calib );
  mp_calib.setInitGyroCalibration( init_gyro_calib );  
  mp_calib.setGravityMagnitude(9.80665);
  mp_calib.enableVerboseOutput(true);
  mp_calib.enableAccUseMeans(false);
  //mp_calib.setGyroDataPeriod(0.01);
  mp_calib.calibrateAccGyro(acc_data, gyro_data );
  //mp_calib.getAccCalib().save("test_imu_acc.calib");
  //mp_calib.getGyroCalib().save("test_imu_gyro.calib");

  cw << std::fixed << std::setprecision(8);
  const Vector3d &abv = mp_calib.getAccCalib().getBiasVector();
  cw << "ACC_BIAS (" << abv[0] << " " << abv[1] << " " << abv[2] << ")" << std::endl;
  const Matrix3d &abg = mp_calib.getAccCalib().getScaleMatrix();
  cw << "ACC_GAIN (" << abg(0, 0) << " " << abg(1, 1) << " " << abg(2, 2) << ")" << std::endl;
  const Matrix3d &abs = mp_calib.getAccCalib().getMisalignmentMatrix();
  cw << "ACC_SKEW (" << abs(0, 0) << " " << abs(0, 1) << " " << abs(0, 2) << " "
                     << abs(1, 0) << " " << abs(1, 1) << " " << abs(1, 2) << " "
                     << abs(2, 0) << " " << abs(2, 1) << " " << abs(2, 2) << ")"
                     << std::endl;
  const Vector3d &gbv = mp_calib.getGyroCalib().getBiasVector();
  cw << "GYR_BIAS (" << gbv[0] << " " << gbv[1] << " " << gbv[2] << ")" << std::endl;
  const Matrix3d &gbg = mp_calib.getGyroCalib().getScaleMatrix();
  cw << "GYR_GAIN (" << gbg(0, 0) << " " << gbg(1, 1) << " " << gbg(2, 2) << ")" << std::endl;
  const Matrix3d &gbs = mp_calib.getGyroCalib().getMisalignmentMatrix();
  cw << "GYR_SKEW (" << gbs(0, 0) << " " << gbs(0, 1) << " " << gbs(0, 2) << " "
                     << gbs(1, 0) << " " << gbs(1, 1) << " " << gbs(1, 2) << " "
                     << gbs(2, 0) << " " << gbs(2, 1) << " " << gbs(2, 2) << ")"
                     << std::endl;

  return 0;
}
