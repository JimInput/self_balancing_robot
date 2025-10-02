#pragma once

class MPU6050 {
public:
    MPU6050() = default;
    void begin(const int address);
    void setup();
    void calculate_IMU_error(const int N);
    void update_measurements();
    void update_angles(float dt);
    void print_angles(unsigned long now);
    void mpuWrite(uint8_t reg, uint8_t val);
    uint8_t mpuRead(uint8_t reg);
    void mpu_configure_filters();
    void print_roll(unsigned long now);
    void print_pitch(unsigned long now);

    float get_roll();
    float get_pitch();
    float get_yaw();
    float get_pitch_rate();

    float get_acc_err_x();
    float get_acc_err_y();
    float get_gyro_err_x();
    float get_gyro_err_y();
    float get_gyro_err_z();

private:
    int address_;
    float acc_x_, acc_y_, acc_z_;
    float gyro_x_, gyro_y_, gyro_z_;
    float gyro_x_raw_, gyro_y_raw_, gyro_z_raw_;
    float gyro_x_filt_, gyro_y_filt_, gyro_z_filt_;
    float gx_hist_[3], gy_hist_[3], gz_hist_[30];
    float gyro_bx_, gyro_by_, gyro_bz_;
    float gyro_lpf_fc_hz_; // low-pass cutoff (Hz)
    float bias_alpha_; // how fast bias adapts when still (small = slow)
    float still_thresh_dps_; // consider still if |rate| < this (deg/sec)
    float acc_angle_x_, acc_angle_y_, gyro_angle_x_, gyro_angle_y_, gyro_angle_z_;
    float roll_, pitch_, yaw_;
    float acc_err_x_, acc_err_y_, gyro_err_x_, gyro_err_y_, gyro_err_z_;
    unsigned long time_since_last_gyro_print_;
};
