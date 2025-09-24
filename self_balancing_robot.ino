// arduino-cli compile --fqbn arduino:avr:uno ./
// arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ./
// usbipd attach --wsl "Ubuntu" --auto-attach --busid 2-3
#include <Arduino.h>
#include <Wire.h>

#include "include/globals.h"
#include "include/stepper_motor.h"
#include "include/mpu.h"

unsigned long now, prev, last_gyro_print;
float dt;

StepperMotor master;
MPU6050 gyro;

void setup() {
    pinMode(0, INPUT_PULLUP);  // release RX
    pinMode(1, INPUT_PULLUP);  // release TX   
    Serial.begin(ArduinoConstants::BAUD_RATE);
    Wire.begin();
    delay(1500);
    Serial.println(F("BOOT OK @ 9600"));

    Wire.setClock(WireConstants::CLOCK_RATE);
    Wire.setWireTimeout(3000, true);
    
    master.begin(StepperConstants::STEPS_PER_REVOLUTION, StepperConstants::STEP_PIN, StepperConstants::MASTER_DIR_PIN);
    gyro.begin(MPUConstants::MPU_ADDRESS);

    gyro.setup();
    gyro.calculate_IMU_error(200);
    master.set_speed(200.0f);
    //
    prev = micros();
}

void update_time() {
    now = micros();
    dt = (now - prev) * 1e-6f;
    prev = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
}

void loop() {
    gyro.update_measurements();
    
    // --- compute dt --- //
    update_time();
    // --- compute dt --- //
    
    // --- GYRO --- //
    gyro.update_angles(dt);
    if (now - last_gyro_print >= 50000) {
        gyro.print_angles();
        last_gyro_print = now;
    }
    // --- GYRO --- //
    
    // --- MOTORS --- //
    digitalWrite(StepperConstants::MASTER_DIR_PIN, HIGH);
    digitalWrite(StepperConstants::SLAVE_DIR_PIN, LOW);
    //
    master.run(dt);
    // --- MOTORS --- //
}

