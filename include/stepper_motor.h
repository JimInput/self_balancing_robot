#pragma once

#include "Arduino.h"

class StepperMotor {
public:
    StepperMotor() = default;
    void begin(int steps_per_rev, int step_pin, int dir_pin);
    void set_speed(float sps);
    void run(float dt);

private:
    int steps_per_rev_;
    int step_pin_;
    int dir_pin_;
    
    volatile uint8_t* step_port_;
    uint8_t step_bitmask_;

    volatile uint8_t* dir_port_;
    uint8_t dir_bitmask_;

    float speed_sps_;
    float frac_steps_;
    int step_pulse_high_us_;
};
