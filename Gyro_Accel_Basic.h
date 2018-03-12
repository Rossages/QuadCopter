/*Header file for Gyro_Accel_Basic.ino*/

#ifndef GYRO_ACCEL_BASIC_H_
#define GYRO_ACCEL_BASIC_H_




// This will set up the co-ordinate system 
// TODO: Maybe Change this to a class! Might be easier and more convienents
struct GyroMeas {
  
  float Yaw;
  float Pitch;
  float Roll;
  uint32_t Time;
  
};

struct AccelMeas {
  
    float Accel_x;
    float Accel_y;
    float Accel_z;
    
};

struct GyroMeas Gyro; // This sets up Gyro as a class of Measurements.
struct GyroMeas Gyroprev; // for previous measurements.

struct AccelMeas Accel; // This sets up Gyro as a class of Measurements.
struct AccelMeas Accelprev; // for previous measurements.


void Measure_MPU(); // This function will take all the measurements from the MPU6050 chip.

void ImuCal();

#endif /*GYRO_ACCEL_BASIC_H_*/


