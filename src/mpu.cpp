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
    time_since_last_gyro_print_ = 0;

    gyro_x_raw_ = gyro_y_raw_ = gyro_z_raw_ = 0.0f;
    gyro_x_filt_ = gyro_y_filt_ = gyro_z_filt_ = 0.0f;
    gx_hist_[0] = gx_hist_[1] = gx_hist_[2] = 0.0f;
    gy_hist_[0] = gy_hist_[1] = gy_hist_[2] = 0.0f;
    gz_hist_[0] = gz_hist_[1] = gz_hist_[2] = 0.0f;
    gyro_bx_ = gyro_by_ = gyro_bz_ = 0.0f;

    gyro_lpf_fc_hz_ = 30.0f;
    bias_alpha_ = 0.001f;
    still_thresh_dps_ = 1.5f;
}

inline float median3(float a, float b, float c) {
    if (a > b) { float t=a; a=b; b=t; }
    if (b > c) { float t=b; b=c; c=t; }
    if (a > b) { float t=a; a=b; b=t; }
    return b;
}

inline float lpf_alpha_from_dt(float dt, float fc_hz) {
    const float tau = 1.0f / (2.0f * PI * fc_hz);
    return dt / (tau + dt);
}

void MPU6050::mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

void MPU6050::mpu_configure_filters() {
    mpuWrite(0x1B, 0x00);
    mpuWrite(0x1C, 0x00);

    const uint8_t DLPF_CFG = 3; // 3 = 44Hz, 4 = 20Hz, 5 = 10Hz, 6 = 5Hz
    mpuWrite(0x1A, DLPF_CFG);

    mpuWrite(0x19, 9);
}

void MPU6050::setup() {
    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::CONFIG_REGISTER_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(true);

    mpu_configure_filters();

}

void MPU6050::update_measurements() {
    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::ACCEL_X_AXIS_REGISTER_ADDRESS);
    Wire.endTransmission(false);
    Wire.requestFrom(address_, 6, true);
    acc_x_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
    acc_y_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
    acc_z_ = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;

    acc_angle_x_ = (atan2(acc_y_, sqrt(acc_x_*acc_x_ + acc_z_*acc_z_)) * 180 / PI) - acc_err_x_;
    acc_angle_y_ = (atan2(-1*acc_x_, sqrt(acc_y_*acc_y_ + acc_z_*acc_z_))* 180/ PI) - acc_err_y_;

    Wire.beginTransmission(address_);
    Wire.write(MPUConstants::GYRO_X_AXIS_REGISTER_ADDRESS);
    Wire.endTransmission(false);

    Wire.requestFrom(address_, 6, true);
 
    const int16_t gx_raw = (Wire.read() << 8) | Wire.read();
    const int16_t gy_raw = (Wire.read() << 8) | Wire.read();
    const int16_t gz_raw = (Wire.read() << 8) | Wire.read();

    const float gx_meas = gx_raw / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR;
    const float gy_meas = gy_raw / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR;
    const float gz_meas = gz_raw / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR;

    // Keep your original variables (subtract static error only)
    gyro_x_ = gx_meas - gyro_err_x_;
    gyro_y_ = gy_meas - gyro_err_y_;
    gyro_z_ = gz_meas - gyro_err_z_;

    // NEW path: subtract static error + online bias
    const float gx = gx_meas - (gyro_err_x_ + gyro_bx_);
    const float gy = gy_meas - (gyro_err_y_ + gyro_by_);
    const float gz = gz_meas - (gyro_err_z_ + gyro_bz_);

    // median-of-3 histories
    gx_hist_[2]=gx_hist_[1]; gx_hist_[1]=gx_hist_[0]; gx_hist_[0]=gx;
    gy_hist_[2]=gy_hist_[1]; gy_hist_[1]=gy_hist_[0]; gy_hist_[0]=gy;
    gz_hist_[2]=gz_hist_[1]; gz_hist_[1]=gz_hist_[0]; gz_hist_[0]=gz;

    // median outputs feed the LPF
    gyro_x_raw_ = median3(gx_hist_[0], gx_hist_[1], gx_hist_[2]);
    gyro_y_raw_ = median3(gy_hist_[0], gy_hist_[1], gy_hist_[2]);
    gyro_z_raw_ = median3(gz_hist_[0], gz_hist_[1], gz_hist_[2]);
}

float complementary_alpha(float dt, float fc_hz) {
    float tau = 1.0f / (2.0f * PI * fc_hz);
    return tau / (tau + dt);
}

void MPU6050::update_angles(float dt) {
    // 1) One-pole LPF on the medianed gyro
    const float a = lpf_alpha_from_dt(dt, gyro_lpf_fc_hz_);
    gyro_x_filt_ += a * (gyro_x_raw_ - gyro_x_filt_);
    gyro_y_filt_ += a * (gyro_y_raw_ - gyro_y_filt_);
    gyro_z_filt_ += a * (gyro_z_raw_ - gyro_z_filt_);

    // 2) Slow bias adaptation when "still" (use filtered rates)
    if (fabsf(gyro_x_filt_) < still_thresh_dps_ &&
        fabsf(gyro_y_filt_) < still_thresh_dps_ &&
        fabsf(gyro_z_filt_) < still_thresh_dps_) {
        gyro_bx_ += bias_alpha_ * gyro_x_filt_;
        gyro_by_ += bias_alpha_ * gyro_y_filt_;
        gyro_bz_ += bias_alpha_ * gyro_z_filt_;
    }

    // 3) Integrate filtered gyro to angles
    gyro_angle_x_ += gyro_x_filt_ * dt;
    gyro_angle_y_ += gyro_y_filt_ * dt;
    yaw_          += gyro_z_filt_ * dt;

    // 4) Complementary fuse (your original function & variables)
    float alpha = complementary_alpha(dt, 30.0f); // your existing 30 Hz
    roll_  = alpha * gyro_angle_x_ + (1 - alpha) * acc_angle_x_;
    pitch_ = alpha * gyro_angle_y_ + (1 - alpha) * acc_angle_y_;
}

void MPU6050::print_angles(unsigned long now) {
    if (now - time_since_last_gyro_print_ >= ArduinoConstants::DATA_PRINT_US) {
        time_since_last_gyro_print_ = now;
        Serial.print(roll_); Serial.print("/");
        Serial.print(pitch_); Serial.print("/");
        Serial.println(yaw_);
    }
}

void MPU6050::print_roll(unsigned long now) {
    if (now - time_since_last_gyro_print_ >= ArduinoConstants::DATA_PRINT_US) {
        time_since_last_gyro_print_ = now;
        Serial.println(roll_);
    }
}

void MPU6050::print_pitch(unsigned long now) {
    if (now - time_since_last_gyro_print_ >= ArduinoConstants::DATA_PRINT_US) {
        time_since_last_gyro_print_ = now;
        Serial.print(pitch_);
        Serial.print(" | ");
        Serial.println(gyro_y_filt_);
    }
}

float MPU6050::get_pitch() {
    return pitch_;
}

float MPU6050::get_pitch_rate() {
    return gyro_y_filt_;
}

void MPU6050::calculate_IMU_error(const int N) {
    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(address_);
        Wire.write(MPUConstants::ACCEL_X_AXIS_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(address_, 6, true);

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
        Wire.beginTransmission(address_);
        Wire.write(MPUConstants::GYRO_X_AXIS_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(address_, 6, true);

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

