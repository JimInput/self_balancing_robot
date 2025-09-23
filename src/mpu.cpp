#include "Arduino.h"
#include "Wire.h"

#include "../include/mpu.h"
#include "../include/globals.h"

void MPU6050::begin(const int address) {
    address_ = address;
    acc_x_ = 0; acc_y_ = 0; acc_z_ = 0;
    gyro_x_ = 0; gyro_y_ = 0; gyro_z_ = 0;
    acc_angle_x_ = 0; acc_angle_y_ = 0;
    gyro_angle_x_ = 0; gyro_angle_y_ = 0; gyro_angle_z_ = 0;
    roll_ = 0; pitch_ = 0; yaw_ = 0;
    acc_err_x_ = 0; acc_err_y_ = 0; 
    gyro_err_x_ = 0; gyro_err_y_ = 0; gyro_err_z_ = 0;
    time_since_last_gyro_reading_ = 0;
    
    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::CONFIG_REGISTER_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::CONFIG_BANDWIDTH_ADDRESS);
    Wire.write(MPUConstants::BANDWIDTH_42_HZ);
    Wire.endTransmission(true);
   
    this->calculate_IMU_error(MPUConstants::NUM_ERROR_TRIALS);
}
