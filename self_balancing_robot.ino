// arduino-cli compile --fqbn arduino:avr:uno ./
// arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ./
// arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=9600 ./
// usbipd attach --wsl "Ubuntu" --auto-attach --busid 2-3

#include <Arduino.h>
#include <Wire.h>

#include "include/globals.h"
#include "include/stepper_motor.h"
#include "include/mpu.h"

uint32_t now, prev;
uint32_t last_blink = 0;
float dt;
float last_pitch = 0.0f;
constexpr uint32_t CONTROL_US = 5000; // 5 ms
constexpr float DT = 0.005f;
uint32_t t_next = 0;
constexpr bool kDebug = false;

StepperMotor master;
StepperMotor slave;
MPU6050 gyro;

float clamp(float v, float small, float big) {
    if (v >= big) return big;
    if (v <= small) return small;
    return v;
}

float absolute_value(float v) {
    if (v<=0) return -v;
    return v;
}   

float pid_loop(float kp, float kq, float ki, float kd, 
               float curr, float rate,
               float target, float deadband, 
               float sens_max, float sens_min, 
               float max_out, float min_out) 
{
    static float last_angle = 0.0f;
    static float I = 0.0f;

    curr = clamp(curr, sens_min, sens_max);
    float e = target - curr;
    bool in_deadband = fabsf(e) <= deadband;

    float p_d = kp*e + kq*e*fabsf(e) - kd*rate;

    bool sat_hi = (p_d >= max_out - 1e-3f) && (e > 0);
    bool sat_lo = (p_d <= min_out + 1e-3f) && (e < 0);

    if (!in_deadband && !sat_hi && !sat_lo) {
        I += e * DT;

        float i_term = ki * I;
        float i_cap = 0.5f * (max_out - min_out);
        if (i_term > i_cap) { i_term = i_cap; I = i_cap / ki; }
        if (i_term < -i_cap) { i_term = -i_cap; I = -i_cap / ki; }
    }

    float u = p_d + ki * I;

    u = clamp(u, min_out, max_out);
    last_angle = curr;

    return u;
}

void setup() {
    Serial.begin(ArduinoConstants::BAUD_RATE);
    Wire.begin();
    delay(1500);
    
    Serial.println(F("BOOT OK @ 9600"));

    Wire.setClock(WireConstants::CLOCK_RATE);
    Wire.setWireTimeout(3000, true);
    
    master.begin(StepperConstants::STEPS_PER_REVOLUTION, StepperConstants::MASTER_STEP_PIN, StepperConstants::MASTER_DIR_PIN);
    slave.begin(StepperConstants::STEPS_PER_REVOLUTION, StepperConstants::SLAVE_STEP_PIN, StepperConstants::SLAVE_DIR_PIN);
    pinMode(LED_BUILTIN, OUTPUT);

    gyro.begin(MPUConstants::MPU_ADDRESS);

    gyro.setup();
    gyro.calculate_IMU_error(200);
    slave.set_speed(0.0f);
    master.set_speed(0.0f);
    
    t_next = micros() + CONTROL_US;

    // prev = micros();
}

// void update_times() {
//     now = micros();
//     dt = (now - prev) * 1e-6f;
//     prev = now;
//     if (dt <= 0.0f || dt > 0.05f) dt = 0.01f; // limit to at most 100ms delay between loop calls
//     if (now - last_blink > 250000) {
//         last_blink = now;
//         digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//         // Serial.println(gyro.get_pitch());
//         // Serial.print(" | ");
//         // Serial.println(pid_loop(10.0, 0, 2.0, gyro.get_pitch(), 0, 1.5, 45, -45, 1200, -1200));
//         // Serial.println(gyro.get_pitch() + " | " + pid_loop(6.666, 0, 0, gyro.get_pitch(), 0, 1, 70, -70, 600, -600)); 
//     }
// }

void loop() {
    gyro.update_measurements(); // this is first to account for delays in reading from I2C bus.
    const uint32_t now_us = micros();    
    // update_times();
    
    if ((int32_t) (now_us - t_next) >= 0) {
        t_next += CONTROL_US;

        static bool skip_I_this_cycle = false;
        skip_I_this_cycle = ((int32_t)(now_us - t_next) > 0);

        gyro.update_angles(DT);

        const float angle_deg = gyro.get_pitch();
        const float rate_dps = gyro.get_pitch_rate();

        float motor_speed = pid_loop(
            15.0f, 0.0f, 0.12f, 6.0f, 
            angle_deg, rate_dps,
            0, 0.75f, 
            45, -45, 
            1000, -1000
        );
        
        static float last_speed = 0.0f;
        const float MAX_SLEW = 600.0f;
        const float delta = motor_speed - last_speed;
        const float limit = MAX_SLEW * DT;
        if (delta > limit) motor_speed = last_speed + limit;
        if (delta < -limit) motor_speed = last_speed - limit;
        last_speed = motor_speed;
        if (fabsf(gyro.get_pitch()) <= 60) {
            master.set_speed(-motor_speed);
            slave.set_speed(motor_speed);
        } else {
            master.set_speed(0);
            slave.set_speed(0);
        }
        

        master.run(DT);
        slave.run(DT);
    }
}

