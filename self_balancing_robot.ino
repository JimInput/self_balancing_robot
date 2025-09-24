// arduino-cli compile --fqbn arduino:avr:uno ./
// arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ./
// arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=9600 ./
// usbipd attach --wsl "Ubuntu" --auto-attach --busid 2-3

#include <Arduino.h>
#include <Wire.h>

#include "include/globals.h"
#include "include/stepper_motor.h"
#include "include/mpu.h"

unsigned long now, prev;
float dt;

StepperMotor master;
StepperMotor slave;
MPU6050 gyro;

void setup() {
    Serial.begin(ArduinoConstants::BAUD_RATE);
    Wire.begin();
    delay(1500);
    
    Serial.println(F("BOOT OK @ 9600"));

    Wire.setClock(WireConstants::CLOCK_RATE);
    Wire.setWireTimeout(3000, true);
    
    master.begin(StepperConstants::STEPS_PER_REVOLUTION, StepperConstants::MASTER_STEP_PIN, StepperConstants::MASTER_DIR_PIN);
    slave.begin(StepperConstants::STEPS_PER_REVOLUTION, StepperConstants::SLAVE_STEP_PIN, StepperConstants::SLAVE_DIR_PIN);
    
    gyro.begin(MPUConstants::MPU_ADDRESS);

    gyro.setup();
    gyro.calculate_IMU_error(200);
    
    master.set_speed(200.0f);
    slave.set_speed(100.0f);

    prev = micros();
}

void update_times() {
    now = micros();
    dt = (now - prev) * 1e-6f;
    prev = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f; // limit to at most 100ms delay between loop calls
}

void loop() {
    gyro.update_measurements(); // this is first to account for delays in reading from I2C bus.
    
    update_times();
    
    gyro.update_angles(dt);
    gyro.print_angles(now);
    
    master.run(dt);
    slave.run(dt);
}

