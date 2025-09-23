// arduino-cli compile --fqbn arduino:avr:uno ./
// arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ./
// usbipd attach --wsl "Ubuntu" --auto-attach --busid 2-3
#include <Arduino.h>
#include <Wire.h>

#include "include/stepper_motor.h"
#include "include/globals.h"

float acc_x, acc_y, acc_z; // read from chip - private variables for Gyro Class
float gyro_x, gyro_y, gyro_z; // read from chip - private variables for Gyro Class
float acc_angle_x, acc_angle_y, gyro_angle_x, gyro_angle_y, gyro_angle_z; // integrate to calculate
float roll, pitch, yaw; // resultant 3 angles
float acc_err_x, acc_err_y, gyro_err_x, gyro_err_y, gyro_err_z; // initial readings to shift by
unsigned long now, prev;
unsigned long t_gyro_data = 0;
float dt;

StepperMotor master(StepperConstants::STEPS_PER_REVOLUTION, StepperConstants::STEP_PIN, StepperConstants::MASTER_DIR_PIN);

void setup() {
    Serial.begin(ArduinoConstants::BAUD_RATE);
    Wire.begin();
    Wire.setClock(WireConstants::CLOCK_RATE);
    Wire.setWireTimeout(3000, true);

    // mpu.setup()
    Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
    Wire.write(MPUConstants::CONFIG_REGISTER_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
    Wire.write(MPUConstants::CONFIG_BANDWIDTH_ADDRESS);
    Wire.write(MPUConstants::BANDWIDTH_42_HZ);
    Wire.endTransmission(true);

    calculate_IMU_error(2000);
    delay(2);
    // mpu.setup()

    master.set_speed(200.0f);

    prev = micros();
}

void loop() {
    // Gyro.load_measurements()
    Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
    Wire.write(MPUConstants::ACCEL_X_AXIS_REGISTER_ADDRESS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);
    acc_x = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
    acc_y = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
    acc_z = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;

    acc_angle_x = (atan2(acc_y, sqrt(acc_x*acc_x + acc_z*acc_z)) * 180 / PI) - acc_err_x;
    acc_angle_y = (atan2(-1*acc_x, sqrt(acc_y*acc_y + acc_z*acc_z))* 180/ PI) - acc_err_y;

    Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
    Wire.write(MPUConstants::GYRO_X_AXIS_REGISTER_ADDRESS);
    Wire.endTransmission(false);

    Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);
    gyro_x = (Wire.read() << 8 | Wire.read()) / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR - gyro_err_x;
    gyro_y = (Wire.read() << 8 | Wire.read()) / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR - gyro_err_y;
    gyro_z = (Wire.read() << 8 | Wire.read()) / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR - gyro_err_z;
    // gyro.load_measurements()

    // COMPUTE dt
    now = micros();
    dt = (now - prev) * 1e-6f;
    prev = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
    // COMPUTE dt

    // update(dt)
    gyro_angle_x = gyro_angle_x + gyro_x * dt;
    gyro_angle_y = gyro_angle_y + gyro_y * dt;
    yaw = yaw + gyro_z * dt;

    roll = MPUConstants::ALPHA * gyro_angle_x + (1-MPUConstants::ALPHA) * acc_angle_x;
    pitch = MPUConstants::ALPHA * gyro_angle_y + (1-MPUConstants::ALPHA) * acc_angle_y;
    // update(dt)

    // print_angles(now/dt);
    if (now - t_gyro_data >= MPUConstants::GYRO_OUTPUT_EVERY_US) {
        t_gyro_data += MPUConstants::GYRO_OUTPUT_EVERY_US; // 20ms
        Serial.print(roll);
        Serial.print("/");
        Serial.print(pitch);
        Serial.print("/");
        Serial.println(yaw);
    }
    //print_angles(now/dt)

    // --- MOTORS --- //
    digitalWrite(StepperConstants::MASTER_DIR_PIN, HIGH);
    digitalWrite(StepperConstants::SLAVE_DIR_PIN, LOW);

    // run motors
    master.run(dt);

    // --- MOTORS --- //
}

void calculate_IMU_error(const int N) {
    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
        Wire.write(MPUConstants::ACCEL_X_AXIS_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);

        acc_x = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
        acc_y = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;
        acc_z = (Wire.read() << 8 | Wire.read()) / MPUConstants::ACCEL_2G_CONSTANT;

        acc_err_x += ((atan2(acc_y, sqrt(acc_x*acc_x + acc_z*acc_z)) * 180 / PI));
        acc_err_y += ((atan2(-1 * (acc_x), sqrt(acc_y*acc_y + acc_z*acc_z)) * 180 / PI));
        delay(2);
    }

    acc_err_x /= static_cast<float>(N);
    acc_err_y /= static_cast<float>(N);

    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(MPUConstants::MPU_ADDRESS);
        Wire.write(MPUConstants::GYRO_X_AXIS_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(MPUConstants::MPU_ADDRESS, 6, true);

        gyro_x = Wire.read() << 8 | Wire.read();
        gyro_y = Wire.read() << 8 | Wire.read();
        gyro_z = Wire.read() << 8 | Wire.read();

        gyro_err_x += (gyro_x / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR);
        gyro_err_y += (gyro_y / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR);
        gyro_err_z += (gyro_z / MPUConstants::GYRO_SENSITIVITY_SCALE_FACTOR);
        delay(2);
    }

    gyro_err_x /= static_cast<float>(N);
    gyro_err_y /= static_cast<float>(N);
    gyro_err_z /= static_cast<float>(N);

    Serial.print("acc_err_x: ");
    Serial.println(acc_err_x);
    Serial.print("acc_err_y: ");
    Serial.println(acc_err_y);
    Serial.print("gyro_err_x: ");
    Serial.println(gyro_err_x);
    Serial.print("gyro_err_y: ");
    Serial.println(gyro_err_y);
    Serial.print("gyro_err_z: ");
    Serial.println(gyro_err_z);
}

