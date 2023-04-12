#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <thread>

// Define pins for motor 1
#define M1_DIR_PIN 13
#define M1_STEP_PIN 6

// Define pins for motor 2
#define M2_DIR_PIN 26
#define M2_STEP_PIN 19


#define Device_Address 0x68

#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

int fd;
int M1_LOCK = 0, M2_LOCK = 0;

float Acc_x, Acc_y, Acc_z;
float Gyro_x, Gyro_y, Gyro_z;
float Ax = 0, Ay = 0, Az = 0;
float Gx = 0, Gy = 0, Gz = 0;
float roll = 0, pitch = 0, yaw = 0;
float alpha = 0.8;
long long int t_prev, t_now;
float dt;


void MPU6050_Init() {
  wiringPiI2CWriteReg8(fd, SMPLRT_DIV,
                       0x07); /* Write to sample rate register */
  wiringPiI2CWriteReg8(fd, PWR_MGMT_1,
                       0x01);          /* Write to power management register */
  wiringPiI2CWriteReg8(fd, CONFIG, 0); /* Write to Configuration register */
  wiringPiI2CWriteReg8(fd, GYRO_CONFIG,
                       24); /* Write to Gyro Configuration register */
  wiringPiI2CWriteReg8(fd, INT_ENABLE,
                       0x01); /* Write to interrupt enable register */
}

short read_raw_data(int addr) {
  short high_byte, low_byte, value;
  high_byte = wiringPiI2CReadReg8(fd, addr);
  low_byte = wiringPiI2CReadReg8(fd, addr + 1);
  value = (high_byte << 8) | low_byte;
  return value;
}

float get_gyro_bias() {
  const int num_samples = 1000;
  const int sample_interval_ms = 1;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  for (int i = 0; i < num_samples; i++) {
    short gx = wiringPiI2CReadReg16(fd, GYRO_XOUT_H);
    short gy = wiringPiI2CReadReg16(fd, GYRO_YOUT_H);
    short gz = wiringPiI2CReadReg16(fd, GYRO_ZOUT_H);
    sum_x += gx;
    sum_y += gy;
    sum_z += gz;
    delay(sample_interval_ms);
  }
  float avg_x = sum_x / num_samples;
  float avg_y = sum_y / num_samples;
  float avg_z = sum_z / num_samples;
  return sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
}

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
  M1_LOCK = 1;
    // Set the direction pin based on the input
    digitalWrite(M1_DIR_PIN, direction);

    // Step the motor the specified number of times
    for (int i = 0; i < steps; i++) {
        digitalWrite(M1_STEP_PIN, HIGH);
        delayMicroseconds(delay);
        digitalWrite(M1_STEP_PIN, LOW);
        delayMicroseconds(delay);
    }
    M1_LOCK = 0;
}

// Function to control the motor 2
void motor2(int direction, int steps, int delay) {
  M2_LOCK = 1;
    // Set the direction pin based on the input
    digitalWrite(M2_DIR_PIN, direction);

    // Step the motor the specified number of times
    for (int i = 0; i < steps; i++) {
        digitalWrite(M2_STEP_PIN, HIGH);
        delayMicroseconds(delay);
        digitalWrite(M2_STEP_PIN, LOW);
        delayMicroseconds(delay);
    }
    M2_LOCK = 0;
}


void readIMU(){
  while(1){
    Acc_x = read_raw_data(ACCEL_XOUT_H);
    Acc_y = read_raw_data(ACCEL_YOUT_H);
    Acc_z = read_raw_data(ACCEL_ZOUT_H);

    Gyro_x = read_raw_data(GYRO_XOUT_H);
    Gyro_y = read_raw_data(GYRO_YOUT_H);
    Gyro_z = read_raw_data(GYRO_ZOUT_H);

    Ax = Acc_x / 16384.0;
    Ay = Acc_y / 16384.0;
    Az = Acc_z / 16384.0;

    Gx = Gyro_x / 131;
    Gy = Gyro_y / 131;
    Gz = Gyro_z / 131;  

    t_now = micros();
    dt = (t_now - t_prev) / 1000000.0;
    t_prev = t_now;

    roll =
        alpha * (roll + Gx * dt) + (1 - alpha) * (atan2(Ay, Az) * 180 / M_PI);
    pitch = alpha * (pitch + Gy * dt) +
            (1 - alpha) * (atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180 / M_PI);
    yaw = alpha * (yaw + Gz * dt) +
          (1 - alpha) * (atan2(sqrt(Ay * Ay + Az * Az), Ax) * 180 / M_PI);
    printf("\n Roll=%.3f°\tPitch=%.3f°\tYaw=%.3f°", roll, pitch, yaw);
}

}


int main() {

  fd = wiringPiI2CSetup(Device_Address); /*Initializes I2C with device Address*/
  MPU6050_Init();                        /* Initializes MPU6050 */
  setup();

  std::thread t1(motor1, 1, 2000, 1500);  // clockwise
  std::thread t2(motor2, 0, 2000, 1500);  // counterclockwise
  t_prev = micros();
  std::thread t3(readIMU);  

  t1.join();
  t2.join();
  t3.join();
  
  return 0;
}
