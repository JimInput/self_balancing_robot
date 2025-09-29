#include "Arduino.h"

#include "../include/stepper_motor.h"

void StepperMotor::begin(int steps_per_rev, int step_pin, int dir_pin) {
    steps_per_rev_ = steps_per_rev;
    step_pin_ = step_pin;
    dir_pin_ = dir_pin;

    speed_sps_ = 0.0f;
    frac_steps_ = 0.0f;
    step_pulse_high_us_ = 4;

    pinMode(step_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);

    step_bitmask_ = digitalPinToBitMask(step_pin_);
    step_port_ = portOutputRegister(digitalPinToPort(step_pin_));

    dir_bitmask_ = digitalPinToBitMask(dir_pin_);
    dir_port_ = portOutputRegister(digitalPinToPort(dir_pin_));

    *step_port_ &= ~step_bitmask_;
    *dir_port_ &= ~dir_bitmask_;

}

void StepperMotor::set_speed(float sps) {
    static bool last_dir_positive = true;
    bool curr_dir_positive = sps >= 0;

    if (curr_dir_positive != last_dir_positive) {
        last_dir_positive = curr_dir_positive;

        if (curr_dir_positive) {
            *dir_port_ |= dir_bitmask_;
        } else {
            *dir_port_ &= ~dir_bitmask_;
        }
        delayMicroseconds(10);
    }
    speed_sps_ = fabs(sps);
}

void StepperMotor::run(float dt) {
    if (speed_sps_ == 0.0f || dt <= 0) return;

    float steps_due = speed_sps_ * dt + frac_steps_;
    int steps_now = static_cast<int>(steps_due);
    frac_steps_ = steps_due - static_cast<float>(steps_now);

    const int MAX_PULSES_PER_CALL = 32;
    int emit = steps_now;
    if (emit > MAX_PULSES_PER_CALL) {
        // frac_steps_ += static_cast<float>(emit - MAX_PULSES_PER_CALL);
        emit = MAX_PULSES_PER_CALL;
    }

    for (int i = 0; i < emit; ++i) {
        *step_port_ |= step_bitmask_;
        delayMicroseconds(step_pulse_high_us_);
        *step_port_ &= ~step_bitmask_;
        delayMicroseconds(step_pulse_high_us_);
    }
}

