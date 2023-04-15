#include <chrono>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <cmath>
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

// Define stepper outputs
#define MIN_VELOCITY_THRESHOLD 1e-6
#define MAX_STEP_DELAY 1e6
#define STEP_PULSE_WIDTH 1e-6

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
#define MIN_VELOCITY -50
#define MAX_VELOCITY 50

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

float pid_p_gain = 2.898;
float pid_i_gain = 0.001;
float pid_d_gain = 0.00538;
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


class Stepper {
private:
    int _dir_pin, _step_pin, _ppr;
    int position = 0;
    int dx = 1;
    float step_delay = 0;
    float last_step_ts = 0;

public:
  int CW = 1;
  int CCW = 0;
    Stepper(int dir_pin, int step_pin, int ppr){
        wiringPiSetupGpio();
        pinMode(dir_pin, OUTPUT);
        pinMode(step_pin, OUTPUT);
	_dir_pin = dir_pin;
	_step_pin = step_pin;
	_ppr = ppr;
    }

    void set_direction(int direction) {
        digitalWrite(_dir_pin, direction);
        if (direction == CW) {
            dx = 1;
        } else {
            dx = -1;
        }
    }

    void step() {
        digitalWrite(_step_pin, HIGH);
        delay(STEP_PULSE_WIDTH);
        digitalWrite(_step_pin, LOW);
        position += dx;
    }

    void set_velocity(double velocity) {
        if (std::abs(velocity) < MIN_VELOCITY_THRESHOLD) {
            step_delay = MAX_STEP_DELAY;
        } else {
	  step_delay = (((2.0 * M_PI) / _ppr) / abs(velocity)) - (1e-6);
            set_direction(velocity > 0 ? CW : CCW);
        }
	//std::cout << step_delay << std::endl ;
    }

    void loop() {
        float now = micros() / 1e6;
	if (now - last_step_ts >= step_delay) {
            step();
            last_step_ts = now;
	}  
    }
};



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

Stepper r_motor(13, 6, 200);
Stepper l_motor(26, 19, 200);

bool UPDATE_VELOCITY = false;
void update_motors(){
  while(1){
    if(pitch < -30 || pitch > 30){
      l_motor.set_velocity(0);
      r_motor.set_velocity(0);
      continue;
    }
    
    if(pitch > -30 && pitch < 30){
      if(pid_output > MAX_VELOCITY){
	pid_output = MAX_VELOCITY;
      }
      if(pid_output < MIN_VELOCITY){
	pid_output = MIN_VELOCITY;
      }

      printf("%.3f\n",pid_output);
    r_motor.set_velocity(pid_output);
    r_motor.loop();

    l_motor.set_velocity(pid_output);
    l_motor.loop();
    UPDATE_VELOCITY = false;
    }
  }
}

void setup() {
  wiringPiSetupGpio();
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);

  fd = wiringPiI2CSetup(Device_Address); /*Initializes I2C with device Address*/
  MPU6050_Init(); 
}

int main() {
  setup();
  t_prev = micros();

  std::thread mT(update_motors);
  
  float mod_pitch = map(pid_output, -260, 260, -90, 90);
  while (true) {

    readIMU();
    pid_error_temp = pitch - pid_setpoint;
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
    pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);

     
    if (pid_output > speed_m) {
      pid_output = speed_m;
    } else if (pid_output < -speed_m) {
      pid_output = -speed_m;
    }

    pid_last_d_error = pid_error_temp;

    if (pid_output < pid_setpoint + 2 && pid_output > pid_setpoint - 2) {
      pid_output = 0;
    }
    /*if(pid_output > 15 || pid_output < -15) {
      pid_output = 15;
    }*/

    // printf("%.3f\n",pitch);
    if (pitch > 30 || pitch < -30) {
      pid_output = 0;
      pid_i_mem = 0;
      self_balance_pid_setpoint = 0;
    }
  
    UPDATE_VELOCITY = true;
    if(pid_output < MIN_VELOCITY){
      pid_output = MIN_VELOCITY;
    }
    else if (pid_output > MAX_VELOCITY){
      pid_output = MAX_VELOCITY;
    }
    // printf("%f\n",pid_output);

    pid_output_left = pid_output;
    pid_output_right = pid_output;
    /*
    if (pid_setpoint == 0) {
      if (pid_output < 0) {
        self_balance_pid_setpoint += 0.0015;
      }
      if (pid_output > 0) {
        self_balance_pid_setpoint -= 0.0015;
      }
    }
    if (pid_output_left > 0) {
      pid_output_left = (1 / (pid_output_left + 9)) * 5500;
    } else if (pid_output_left < 0) {
      pid_output_left = -(1 / (pid_output_left - 9)) * 5500;
    }

    if (pid_output_right > 0) {
      pid_output_right = (1 / (pid_output_right + 9)) * 5500;
    } else if (pid_output_right < 0) {
      pid_output_right = - (1 / (pid_output_right - 9)) * 5500;
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
    */
    unsigned long nowT = micros();
    accel = v[0];
    prev_pitch = mod_pitch;
    prev_yaw = yaw;
  }
  mT.join();
  return 0;
}
