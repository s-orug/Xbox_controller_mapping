#include <wiringPi.h>
#include <iostream>

// Define pins for the motor
const int STEP_PIN = 17; // GPIO17 (WiringPi pin 0)
const int DIR_PIN = 27;  // GPIO27 (WiringPi pin 2)

// Define stepper motor parameters
const int STEPS_PER_REV = 200;   // Steps per revolution
const int MICROSTEPS = 1;       // Microsteps per step
const float STEP_DELAY = 50000.0;  // Delay in microseconds between each step

int main() {
    // Initialize WiringPi library
    wiringPiSetupGpio();

    // Configure motor pins as output
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    // Set the motor direction (CW or CCW)
    digitalWrite(DIR_PIN, HIGH); // CW

    // Move the motor one revolution
    for (int i = 0; i < STEPS_PER_REV * MICROSTEPS; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEP_DELAY);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEP_DELAY);
    }

    // Set the motor direction (CCW or CW)
    digitalWrite(DIR_PIN, LOW); // CCW

    // Move the motor one revolution
    for (int i = 0; i < STEPS_PER_REV * MICROSTEPS; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEP_DELAY);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEP_DELAY);
    }

    return 0;
}
