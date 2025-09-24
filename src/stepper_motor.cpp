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

    digitalWrite(step_pin_, LOW);
    digitalWrite(dir_pin_, LOW);
}

void StepperMotor::set_speed(float sps) {
    speed_sps_ = sps;
    if (sps > 0) {
        digitalWrite(dir_pin_, HIGH);
    } else {
        digitalWrite(dir_pin_, LOW);
    }
}

void StepperMotor::run(float dt) {
    if (speed_sps_ == 0.0f || dt <= 0) return;

    float steps_due = speed_sps_ * dt + frac_steps_;
    int steps_now = static_cast<int>(steps_due);
    frac_steps_ = steps_due - static_cast<float>(steps_now);

    const int MAX_PULSES_PER_CALL = 8;
    int emit = steps_due;
    if (emit > MAX_PULSES_PER_CALL) {
        frac_steps_ += static_cast<float>(emit - MAX_PULSES_PER_CALL);
        emit = MAX_PULSES_PER_CALL;
    }

    for (int i = 0; i < emit; ++i) {
        digitalWrite(step_pin_, HIGH);
        delayMicroseconds(step_pulse_high_us_);
        digitalWrite(step_pin_, LOW);
        delayMicroseconds(step_pulse_high_us_);
    }
}

