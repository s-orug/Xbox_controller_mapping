#include <wiringPi.h>
#include <thread>

// Define pins for motor 1
#define M1_DIR_PIN 13
#define M1_STEP_PIN 6

// Define pins for motor 2
#define M2_DIR_PIN 26
#define M2_STEP_PIN 19

// Define the delay in microseconds between each step (adjust as needed)
#define STEP_DELAY_MICROSECONDS 2000

// Function to set up the GPIO pins
void setup() {
    // Initialize the wiringPi library
    wiringPiSetupGpio();

    // Set the direction and step pins as output
    pinMode(M1_DIR_PIN, OUTPUT);
    pinMode(M1_STEP_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);
    pinMode(M2_STEP_PIN, OUTPUT);
}

// Function to control the motor 1
void motor1(int direction, int steps, int delay) {
    // Set the direction pin based on the input
    digitalWrite(M1_DIR_PIN, direction);

    // Step the motor the specified number of times
    for (int i = 0; i < steps; i++) {
        digitalWrite(M1_STEP_PIN, HIGH);
        delayMicroseconds(delay);
        digitalWrite(M1_STEP_PIN, LOW);
        delayMicroseconds(delay);
    }
}

// Function to control the motor 2
void motor2(int direction, int steps, int delay) {
    // Set the direction pin based on the input
    digitalWrite(M2_DIR_PIN, direction);

    // Step the motor the specified number of times
    for (int i = 0; i < steps; i++) {
        digitalWrite(M2_STEP_PIN, HIGH);
        delayMicroseconds(delay);
        digitalWrite(M2_STEP_PIN, LOW);
        delayMicroseconds(delay);
    }
}

int main() {
    // Set up the GPIO pins
    setup();

    // Start two threads to control the motors simultaneously
    std::thread t1(motor1, 1, 2000, 1500);  // clockwise
    std::thread t2(motor2, 0, 2000, 1500);  // counterclockwise

    // Wait for both threads to finish
    t1.join();
    t2.join();

    return 0;
}
