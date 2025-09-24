#pragma once

class MPU6050 {
public:
    MPU6050() = default;
    void begin(const int address);
    void setup();
    void calculate_IMU_error(const int N);
    void update_measurements();
    void update_angles(float dt);
    void print_angles();
    void mpuWrite(uint8_t reg, uint8_t val);
    uint8_t mpuRead(uint8_t reg);
    void mpu_configure_filters();

    float get_roll();
    float get_pitch();
    float get_yaw();

    float get_acc_err_x();
    float get_acc_err_y();
    float get_gyro_err_x();
    float get_gyro_err_y();
    float get_gyro_err_z();

private:
    int address_;
    float acc_x_, acc_y_, acc_z_;
    float gyro_x_, gyro_y_, gyro_z_;
    float acc_angle_x_, acc_angle_y_, gyro_angle_x_, gyro_angle_y_, gyro_angle_z_;
    float roll_, pitch_, yaw_;
    float acc_err_x_, acc_err_y_, gyro_err_x_, gyro_err_y_, gyro_err_z_;
    int num_print_calls_; 
};
