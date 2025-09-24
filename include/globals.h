#pragma once

namespace StepperConstants {
    inline constexpr int MASTER_DIR_PIN = 2;
    inline constexpr int MASTER_STEP_PIN = 3;
    inline constexpr int STEPS_PER_REVOLUTION = 200;
    inline constexpr int SLAVE_DIR_PIN = 4;
    inline constexpr int SLAVE_STEP_PIN = 5;
}

namespace ArduinoConstants {
    inline constexpr int BAUD_RATE = 9600;
    inline constexpr int DATA_PRINT_US = 50000;
}

namespace WireConstants {
    inline constexpr int CLOCK_RATE = 400000;
}

namespace MPUConstants {
    inline constexpr int MPU_ADDRESS = 0x68;
    inline constexpr int CONFIG_REGISTER_ADDRESS = 0x6B;
    inline constexpr int CONFIG_BANDWIDTH_ADDRESS = 0x1A;
    inline constexpr int BANDWIDTH_42_HZ = 0x2;
    inline constexpr int ACCEL_X_AXIS_REGISTER_ADDRESS = 0x3B;
    inline constexpr int GYRO_X_AXIS_REGISTER_ADDRESS = 0x43;
    inline constexpr float ACCEL_2G_CONSTANT = 16384.0; // limits accel to +- 2g
    inline constexpr float GYRO_SENSITIVITY_SCALE_FACTOR = 131.0; // allow gyro to register angular input of 250 deg/sec
    inline constexpr float ALPHA = 0.96;
    inline constexpr int NUM_ERROR_TRIALS = 200;
}
