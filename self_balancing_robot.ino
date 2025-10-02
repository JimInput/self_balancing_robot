// arduino-cli compile --fqbn arduino:avr:uno ./
// arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ./
// arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=9600 ./
// usbipd attach --wsl "Ubuntu" --auto-attach --busid 2-3

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

#include "include/globals.h"
#include "include/mpu.h"

uint32_t now;
uint32_t last_blink = 0;

constexpr uint32_t CONTROL_US = 5000; // 5ms loop
constexpr float DT = 0.005f;

// --- Sign-agnostic trim learner state ---
float ema_abs_speed = 0.0f;
float ema_cmd = 0.0f;
static float sign_gain = -1.0f;
static float last_cost = 1e9f;
static uint32_t adapt_timer = 0;

constexpr float TRIM_PROBE_DEG = 0.02f;
constexpr float TRIM_LEARN_GAMMA = 0.00005f;
constexpr uint32_t ADAPT_PERIOD_US = 300000;

float angle_trim_deg = -0.5f;
float ema_motor_speed = 0.0f;
float ema_rate_dps = 0.0f;

uint32_t t_next = 0;

AccelStepper master(AccelStepper::DRIVER, StepperConstants::MASTER_STEP_PIN, StepperConstants::MASTER_DIR_PIN);
AccelStepper slave(AccelStepper::DRIVER, StepperConstants::SLAVE_STEP_PIN, StepperConstants::SLAVE_DIR_PIN);
MPU6050 gyro;

float clamp(float v, float small, float big) {
    if (v >= big) return big;
    if (v <= small) return small;
    return v;
}

float pid_loop(float kp, float kq, float ki, float kd, 
               float curr, float rate,
               float target, float deadband, 
               float sens_max, float sens_min, 
               float max_out, float min_out,
               float dt) 
{
    static float I = 0.0f;

    curr = clamp(curr, sens_min, sens_max);
    float e = target - curr;
    bool in_deadband = fabsf(e) <= deadband;

    float kd_eff = kd;
    if (fabsf(curr) < 3.0f) {
        kd_eff *= 1.8f;
    }

    float p_term = kp*e + kq*e*fabsf(e);

    const float RATE_CLAMP = 600.0f;
    float d_meas = clamp(rate, -RATE_CLAMP, RATE_CLAMP);
    float d_term = -kd_eff * d_meas;

    float pre_u = p_term + d_term;
    
    bool sat_hi = (pre_u >= max_out - 1e-3f) && (e > 0);
    bool sat_lo = (pre_u <= min_out + 1e-3f) && (e < 0);
    bool allow_I = !in_deadband && !sat_hi && !sat_lo && (fabsf(curr) < 15.0f);

    if (allow_I) {
        I += e * dt;

        float i_term = ki * I;
        float i_cap = 0.5f * (max_out - min_out);
        if (i_term > i_cap) { i_term = i_cap; I = i_cap / ki; }
        if (i_term < -i_cap) { i_term = -i_cap; I = -i_cap / ki; }
    } else {
        //bleed I
        I *= 0.90;
    }

    float u = pre_u + ki * I;

    return clamp(u, min_out, max_out);
}

void setup() {
    Serial.begin(ArduinoConstants::BAUD_RATE);
    Wire.begin();
    delay(1500);
    
    Serial.println(F("BOOT OK @ 9600"));

    Wire.setClock(WireConstants::CLOCK_RATE);
    Wire.setWireTimeout(3000, true);
   
    master.enableOutputs();
    slave.enableOutputs();

    master.setMaxSpeed(6000);
    slave.setMaxSpeed(6000);
    master.setAcceleration(80000);
    slave.setAcceleration(80000);

    master.setCurrentPosition(0);
    slave.setCurrentPosition(0);
    
    pinMode(LED_BUILTIN, OUTPUT);

    gyro.begin(MPUConstants::MPU_ADDRESS);

    gyro.setup();
    gyro.calculate_IMU_error(200);
    
    now = micros();
    t_next = now + CONTROL_US;
}

// void update_times() {
//     now = micros();
//     dt = (now - prev) * 1e-6f;
//     prev = now;
//     if (dt <= 0.0f || dt > 0.05f) dt = 0.01f; // limit to at most 100ms delay between loop calls
//     if (now - last_blink > 250000) {
//         last_blink = now;
//         digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//     }
// }

void loop() {
    now = micros();

    if ((int32_t)(now-t_next) < 0) {
        master.runSpeed();
        slave.runSpeed();
        return;
    }
    t_next += CONTROL_US;

    // Blink (non-blocking)
    if (now - last_blink > 250000) {
        last_blink = now;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.print(F("trim=")); Serial.print(angle_trim_deg, 3);
        Serial.print(F("  ema_u=")); Serial.print(ema_motor_speed, 1);
        Serial.print(F("  |u|_ema=")); Serial.print(ema_abs_speed, 1); // if using B
        Serial.print(F("  cmd_ema=")); Serial.println(ema_cmd, 1);
     }

    gyro.update_measurements(); // this is first to account for delays in reading from I2C bus.
    // update_times(); 
    gyro.update_angles(DT);

    const float angle_deg_raw = gyro.get_pitch();
    const float rate_dps = gyro.get_pitch_rate();

    const float angle_deg = angle_deg_raw - angle_trim_deg;

    float motor_speed = pid_loop(
        10.0f, 0.40f, 0.1f, 8.0f, 
        angle_deg, rate_dps,
        0, 1.0f, 
        35, -35, 
        6000, -6000,
        DT
    );
    // Smoothness/learning gates
    const float ema_alpha = DT / 1.5f;
    ema_cmd       += ema_alpha * (motor_speed - ema_cmd);
    ema_abs_speed += ema_alpha * (fabsf(motor_speed) - ema_abs_speed);

    bool near_upright = fabsf(angle_deg) < 3.0f && fabsf(ema_rate_dps) < 8.0f; 
    bool not_sat = fabsf(motor_speed) < 0.8f * 6000.0f;
    bool calm_cmd     = fabsf(ema_cmd) < 0.25f * 6000.0f;
    bool learn_window = near_upright && not_sat && calm_cmd;



    // Tiny sinusoidal probe around current trim (0.5 Hz)
    if (learn_window) {
        float probe = TRIM_PROBE_DEG * sinf(2.0f * PI * 0.5f * (now * 1e-6f));
        angle_trim_deg = clamp(angle_trim_deg + probe, -5.0f, 5.0f);
    }

    // Periodic adaptation step
    if (learn_window && (now - adapt_timer) >= ADAPT_PERIOD_US) {
        adapt_timer = now;

        float cost = ema_abs_speed;             // we want this to decrease

        // If last step made it worse (beyond small hysteresis), flip the push direction
        if (cost > last_cost + 1.0f) {
            sign_gain = -sign_gain;
        }
        last_cost = cost;

        // Gradient-like update toward lower avg speed (sign auto-handled)
        angle_trim_deg += sign_gain * (-TRIM_LEARN_GAMMA) * ema_motor_speed;
        angle_trim_deg = clamp(angle_trim_deg, -5.0f, 5.0f);
    } else if (!learn_window) {
        last_cost = 1e9f;                        // reset between learning windows
    }  

    if (fabsf(angle_deg) < 0.6f && fabsf(rate_dps) < 6.0f && fabsf(motor_speed) < 80.0f) 
        motor_speed = 0.0f;

    if (fabsf(angle_deg) > 35.0f)
        motor_speed = 0.0f;

    master.setSpeed(-motor_speed);
    slave.setSpeed(motor_speed);

    master.runSpeed();
    slave.runSpeed();

  }

