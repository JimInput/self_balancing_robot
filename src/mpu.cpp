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
}

void MPU6050::setup() {
    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::CONFIG_REGISTER_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::CONFIG_BANDWIDTH_ADDRESS);
    Wire.write(MPUConstants::BANDWIDTH_42_HZ);
    Wire.endTransmission(true);

    this->calculate_IMU_error(MPUConstants::NUM_ERROR_TRIALS);
    delay(2);
}

void MPU6050::update_measurements() {
    Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
    Wire.write(MPUConstants::ACCEL_X_AXIS_REGISTER_ADDRESS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);
    acc_x_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
    acc_y_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
    acc_z_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;

    acc_angle_x_ = (atan2(acc_y_, sqrt(acc_x_*acc_x_ + acc_z_*acc_z_)) * 180 / PI) - acc_err_x_;
    acc_angle_y_ = (atan2(-1*acc_x_, sqrt(acc_y_*acc_y_ + acc_z_*acc_z_))* 180/ PI) - acc_err_y_;

    Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
    Wire.write(MPUConstants::GYRO_X_AXIS_REGISTER_ADDRESS);
    Wire.endTransmission(false);

    Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);
    gyro_x_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR - gyro_err_x_;
    gyro_y_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR - gyro_err_y_;
    gyro_z_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR - gyro_err_z_;
}

void MPU6050::update_angles(float dt) {
    gyro_angle_x_ += gyro_x_ * dt;
    gyro_angle_y_ += gyro_y_ * dt;
    yaw_ += gyro_z_ * dt;

    roll_ = MPUConstants::ALPHA * gyro_angle_x_ + (1-MPUConstants::ALPHA) * acc_angle_x_;
    pitch_ = MPUConstants::ALPHA * gyro_angle_y_ + (1-MPUConstants::ALPHA) * acc_angle_y_;
}

void MPU6050::print_angles(unsigned long now) {
    if (now - time_since_last_gyro_reading_ >= MPUConstants::GYRO_OUTPUT_EVERY_US) {
        time_since_last_gyro_reading_ += MPUConstants::GYRO_OUTPUT_EVERY_US; // 20ms
        Serial.print(roll_);
        Serial.print("/");
        Serial.print(pitch_);
        Serial.print("/");
        Serial.println(yaw_);
    }
}

void MPU6050::calculate_IMU_error(const int N) {
    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
        Wire.write(MPUConstants::ACCEL_X_AXIS_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);

        acc_x_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
        acc_y_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
        acc_z_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;

        acc_err_x_ += ((atan2(acc_y_, sqrt(acc_x_*acc_x_ + acc_z_*acc_z_)) * 180 / PI));
        acc_err_y_ += ((atan2(-1 * (acc_x_), sqrt(acc_y_*acc_y_ + acc_z_*acc_z_)) * 180 / PI));
        delay(2);
    }

    acc_err_x_ /= static_cast<float>(N);
    acc_err_y_ /= static_cast<float>(N);

    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
        Wire.write(MPUConstants::GYRO_X_AXIS_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);

        gyro_x_ = Wire.read() << 8 | Wire.read();
        gyro_y_ = Wire.read() << 8 | Wire.read();
        gyro_z_ = Wire.read() << 8 | Wire.read();

        gyro_err_x_ += (gyro_x_ / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR);
        gyro_err_y_ += (gyro_y_ / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR);
        gyro_err_z_ += (gyro_z_ / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR);
        delay(2);
    }

    gyro_err_x_ /= static_cast<float>(N);
    gyro_err_y_ /= static_cast<float>(N);
    gyro_err_z_ /= static_cast<float>(N);

    Serial.print("acc_err_x: ");
    Serial.println(acc_err_x_);
    Serial.print("acc_err_y: ");
    Serial.println(acc_err_y_);
    Serial.print("gyro_err_x: ");
    Serial.println(gyro_err_x_);
    Serial.print("gyro_err_y: ");
    Serial.println(gyro_err_y_);
    Serial.print("gyro_err_z: ");
    Serial.println(gyro_err_z_);
}

