#include <chrono>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <stdint.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

// Define pins for motor 1

#define mapRange(a1, a2, b1, b2, s) (b1 + (s - a1) * (b2 - b1) / (a2 - a1))

// Define pins for motor 1
#define M1_DIR_PIN 13
#define M1_STEP_PIN 6

// Define pins for motor 2
#define M2_DIR_PIN 26
#define M2_STEP_PIN 19

// Define pins for IMU

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

#define alpha1 0.03

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
float accel = 0, velocity = 0;

float k[4] = {-10,-10.6897,70.5556,16.7775};//{18.8 ,  -7746.3  , 40.1 ,  -70};//22 ,-7458.4,  40  ,-60.8};

//  gains for yaw | yaw_dot
float k1[2] = {-2,   -0.3};

float u[2] = {0, 0};
float v[2] = {0, 0};
// MATLAB tuned PID params.
// Controller Parameters: P = 2.968, I = 9.758, D = 0.1838, N = 7181

// PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
// PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);

float pid_p_gain = 2.968;
float pid_i_gain = 9.758;
float pid_d_gain = 0.1838;
float turning_speed = 400; // Turning speed (900)
float max_target_speed = 1400;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint = 1, gyro_input, pid_output,
                                 pid_last_d_error;
float pid_output_left, pid_output_right;
int speed_m = 2500; // max 2500
float pickup = 0.009;
float current_pitch = 0;
float prev_pitch = 0;
float prev_yaw = 0;

float ax_lp = 0, ay_lp = 0, az_lp = 0;
float gx_hp = 0, gy_hp = 0, gz_hp = 0;
float gx_p = 0, gy_p = 0;

int left_motor;
int right_motor;

int DELAY = 2;

int throttle_left_motor = 200;  // initial throttle for left motor
int throttle_right_motor = 200; // initial throttle for right motor
int throttle_counter_left_motor = 0;
int throttle_counter_right_motor = 0;
int throttle_left_motor_memory = 0;
int throttle_right_motor_memory = 0;

bool DRIVE_MOTORS = false;

// PID

// PID motorsPID(2.968, 9.758, 0.1838, 0);
// PID anglePID(2.968, 9.758, 0.1838, 0);

class Stepper {
  public:

  

int getTicksPerPulse(float velocity, int32_t ticksPerSecond, uint32_t pulsesPerRevolution, uint32_t pulseWidth) {
  if (abs(velocity) < 1e-3) {
    return UINT32_MAX;
  } else {
    return (uint32_t)(6.28318530718 * ticksPerSecond / (abs(velocity) * pulsesPerRevolution)) - pulseWidth;
  }  
}

Stepper(int enablePin, int dirPin, int stepPin, uint32_t ticksPerSecond, uint32_t pulsesPerRevolution, uint32_t pulseWidth) {
  // this->enablePin = enablePin;
  this->dirPin = dirPin;
  this->stepPin = stepPin;
  this->ticksPerSecond = ticksPerSecond;
  this->pulsesPerRevolution = pulsesPerRevolution;
  this->pulseWidth = pulseWidth;  
}

void init() {
  // wiringPiSetupGpio();
  // pinMode(enablePin, OUTPUT);
  // pinMode(dirPin, OUTPUT);
  // pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);

  
  this->currentTick = 0;
}

void setEnabled(bool enabled) {
  digitalWrite(enablePin, enabled ? LOW : HIGH);
}

void setVelocity(float velocity) {
  ticksPerPulse = getTicksPerPulse(velocity, ticksPerSecond, pulsesPerRevolution, pulseWidth);
  digitalWrite(dirPin, velocity > 0 ? HIGH : LOW);
}

void tick() {
  if (currentTick >= ticksPerPulse) {
    currentTick = 0;
  }
  if (currentTick == 0) {
    digitalWrite(stepPin, HIGH);
  } else if (currentTick == pulseWidth) {    
    digitalWrite(stepPin, LOW);
  }
  currentTick++; 
}
  private:
    int enablePin;
    int dirPin;
    int stepPin;
    uint32_t ticksPerSecond;
    uint32_t pulsesPerRevolution;
    uint32_t pulseWidth;
    volatile uint32_t currentTick;
    volatile uint32_t ticksPerPulse;
};

// Stepper m1(-1, M1_DIR_PIN, M1_STEP_PIN, 100000, 200, 10);
// GENERAL

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//  IMU

short read_raw_data(int addr) {
  short high_byte, low_byte, value;
  high_byte = wiringPiI2CReadReg8(fd, addr);
  low_byte = wiringPiI2CReadReg8(fd, addr + 1);
  value = (high_byte << 8) | low_byte;
  return value;
}

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

void filter() {
  // low pass;
  ax_lp = (1 - alpha) * (Ax) + (alpha * (ax_lp));
  ay_lp = (1 - alpha) * (Ay) + (alpha * (ay_lp));
  az_lp = (1 - alpha) * (Az) + (alpha * (az_lp));

  // high pass;
  gx_hp = (1 - alpha) * gx_hp + (1 - alpha) * ((Gx)-gx_p);
  gy_hp = (1 - alpha) * gy_hp + (1 - alpha) * ((Gy)-gy_p);

  gx_p = gx_hp;
  gy_p = gy_hp;
}

void readIMU() {
  while (1) {
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

    filter();
    current_pitch = current_pitch - (gy_hp * dt);
    float acc_angle = atan2(ax_lp, sqrt(az_lp * az_lp)) * (180 / 3.1415);

    current_pitch = (1 - alpha1) * current_pitch +
                    (alpha1) * (acc_angle); // complementary filter

    roll =
        alpha * (roll + Gx * dt) + (1 - alpha) * (atan2(Ay, Az) * 180 / M_PI);
    pitch = current_pitch; // alpha * (pitch + Gy * dt) +
                           //(1 - alpha) * (atan2(Ax, sqrt(Ay * Ay + Az * Az)) *
                           // 180 / M_PI);
    yaw = alpha * (yaw + Gz * dt) +
          (1 - alpha) * (atan2(sqrt(Ay * Ay + Az * Az), Ax) * 180 / M_PI);

    // printf("\n Roll=%.3f°\tPitch=%.3f°\tYaw=%.3f°", roll, pitch, yaw);
    // printf("\n Current Pitch=%.3f°\tPitch=%.3f°\n", current_pitch, pitch);
  }
}

// STEPPER

void leftMotorPulse(float d1, float d2) {
  digitalWrite(M1_STEP_PIN, HIGH);
  //std::this_thread::sleep_for(std::chrono::microseconds(DELAY));
  delay(d1); // {0, 1, 2, 2.5}
  digitalWrite(M1_STEP_PIN, LOW);
  delay(0.75); // {1. 0.75, 0.8, 0.85}
}

void rightMotorPulse(float d1, float d2) {
  digitalWrite(M2_STEP_PIN, HIGH);
  //std::this_thread::sleep_for(std::chrono::microseconds(DELAY));
  delay(d1);
  digitalWrite(M2_STEP_PIN, LOW);
  delay(0.75);
}

void leftMotorControl() {
  while (true) {
    if (1==0){
    throttle_counter_left_motor++;
    if (throttle_counter_left_motor > throttle_left_motor_memory) {
      throttle_counter_left_motor = 0;
      throttle_left_motor_memory = throttle_left_motor;
      if (throttle_left_motor_memory < 0) {
        digitalWrite(M1_DIR_PIN, LOW);
        throttle_left_motor_memory *= -1;
      } else
        digitalWrite(M1_DIR_PIN, HIGH);
    } else if (throttle_counter_left_motor == 1) {
      digitalWrite(M1_STEP_PIN, HIGH);
      delay(DELAY);
    } else if (throttle_counter_left_motor == 2) {
      digitalWrite(M1_STEP_PIN, LOW);
      delay(2);
    }
    DRIVE_MOTORS = false;
    }
  }
}


void rightMotorControl() {
  while (true) {
    // DELAY = 2500;
    //   rightMotorPulse();
    // leftMotorPulse();
  }
}

void updateMotors(float velocity){
  float delay = mapRange(0, 800, 3, 0, abs(velocity));
  float delay2 = mapRange(0, 800, 0.75, 1, abs(velocity));

  digitalWrite(M1_DIR_PIN, HIGH);
  digitalWrite(M2_DIR_PIN, HIGH);
  
  rightMotorPulse(delay, delay2);
  leftMotorPulse(delay, delay2);
  std::cout << delay << std::endl;
}

void updateVelocity(unsigned long nowT, float accel){
  // std::cout << v[0] << std::endl;
  static unsigned long timestamp = micros();
  if(nowT - timestamp < 50){
    return;
  }
  float _dt = ((float) (nowT - timestamp)) * 1e-6;
  velocity = accel*dt;
  updateMotors(velocity);
  timestamp = nowT;
  updateMotors(velocity);
  // std::cout << velocity << std::endl;
}

void setup() {
  // Initialize the wiringPi library
  wiringPiSetupGpio();

  // Set the direction and step pins as output
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);

  fd = wiringPiI2CSetup(Device_Address); /*Initializes I2C with device Address*/
  MPU6050_Init();                        /* Initializes MPU6050 */
  // m1.setVelocity(100);
  // m1.tick();
}

int main() {
  setup();
  t_prev = micros();

  std::thread imuT(readIMU);
  std::thread leftMotorT(leftMotorControl);
  std::thread rightMotorT(rightMotorControl);

  float mod_pitch = map(pid_output, -260, 260, -90, 90);
  while (true) {

    pid_error_temp = pitch - pid_setpoint;
    // std::cout << pid_error_temp << std::endl;
    if (pid_output > 10 || pid_output < -10) {
      pid_error_temp += pid_output * 0.015;
    }

    pid_i_mem += pid_i_gain * pid_error_temp;
    if (pid_i_mem > speed_m) {
      pid_i_mem = speed_m;
    } else if (pid_i_mem < -speed_m) {
      pid_i_mem = -speed_m;
    }
    // Calculate the PID output value
     pid_output = pid_p_gain * pid_error_temp +
                  pid_d_gain * (pid_error_temp - pid_last_d_error);
    // float pid_angle = anglePID.getControl(current_pitch, dt/100000);
    // pid_output = motorsPID.getControl(current_pitch, dt/100000);
     //std::cout << map(pid_output, -260, 260, -90, 90) << "\t" << current_pitch << std::endl;
    delay(10);
    if (pid_output > speed_m) {
      pid_output = speed_m;
    } else if (pid_output < -speed_m) {
      pid_output = -speed_m;
    }

    pid_last_d_error = pid_error_temp;

    if (pid_output < pid_setpoint + 1 && pid_output > pid_setpoint - 1) {
      pid_output = 0;
    }
    /*if(pid_output > 15 || pid_output < -15) {
      pid_output = 15;
    }*/

    if (pitch > 30 || pitch < -30) {
      pid_output = 0;
      pid_i_mem = 0;
      self_balance_pid_setpoint = 0;
    }
    // printf("%d\n",pid_error_tem);
    // printf("%f\n",pid_output);

    pid_output_left = pid_output;
    pid_output_right = pid_output;

    if (pid_setpoint == 0) {
      if (pid_output < 0) {
        self_balance_pid_setpoint += 0.0015;
      }
      if (pid_output > 0) {
        self_balance_pid_setpoint -= 0.0015;
      }
    }
    if (pid_output_left > 0) {
      pid_output_left = speed_m - (1 / (pid_output_left + 9)) * 5500;
    } else if (pid_output_left < 0) {
      pid_output_left = -speed_m - (1 / (pid_output_left - 9)) * 5500;
    }

    if (pid_output_right > 0) {
      pid_output_right = speed_m - (1 / (pid_output_right + 9)) * 5500;
    } else if (pid_output_right < 0) {
      pid_output_right = -speed_m - (1 / (pid_output_right - 9)) * 5500;
    }

    // Calculate the needed pulse time for the left and right stepper motor
    // controllers
    if (pid_output_left > 0) {
      left_motor = speed_m - pid_output_left;
    } else if (pid_output_left < 0) {
      left_motor = -speed_m - pid_output_left;
    } else {
      left_motor = 0;
    }

    if (pid_output_right > 0) {
      right_motor = speed_m - pid_output_right;
    } else if (pid_output_right < 0) {
      right_motor = -speed_m - pid_output_right;
    } else {
      right_motor = 0;
    }
    // std::cout << left_motor << "\t" << right_motor << std::endl;
    throttle_left_motor = left_motor;
    throttle_right_motor = right_motor;
    DELAY = map(((left_motor + right_motor) >> 2), -600, 600, 2, 10);

    float pitch_dot = (current_pitch - prev_pitch)/dt;
    float yaw_dot = (yaw - prev_yaw)/dt;
    //float angle_dot = (0);
    u[0] = ( k[0]*(prev_pitch-mod_pitch) //angle
	     +k[1]*(-mod_pitch) // position
	     +k[2]*(DELAY) // velocity
	     +k[3]*(-pitch_dot)); // angular velocity
          
    u[1] = (k1[0]*yaw +k1[1]*yaw_dot) ;
    v[0] = (0.5 *(u[0] + u[1])); // for right motor
    v[1] = (0.5 *(u[0] + u[1])); // for left motor

    unsigned long nowT = micros();
    accel = v[0];
    updateVelocity(nowT, accel);
    
    // updateControl(nowT);
    
    // std::cout << "U:\t" << u[0] << "\t" << u[1] << std::endl;
    // std::cout << "V:\t" << v[0] << "\t" << v[1] << std::endl;
    // std::cout << DELAY << std::endl;
    // while (loop_t0imer > micros())
    //   ;
    // loop_timer += 4000;
    prev_pitch = mod_pitch;
    prev_yaw = yaw;
  }

  imuT.join();
  leftMotorT.join();
  rightMotorT.join();

  return 0;
}
