#include <Wire.h>

int gyro_address = 0x68;
int acc_calibration_value = -7902;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 38;
float pid_i_gain = 1;
float pid_d_gain = 36;
float turning_speed = 400;                                    //Turning speed (900)
float max_target_speed = 1400;                                //Max target speed (1500)

byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
int speed_m = 1000; //max 2500
float pickup = 0.009; 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;

  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  // internally CNC shiled pins are connected to 2,3,5,6 for step and dir
  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(6, OUTPUT);                                                       //Configure digital poort 6 as output

  for (receive_counter = 0; receive_counter < 500; receive_counter++) {     //Create 500 loops
    if (receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));       //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();           //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();         //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (Serial.available()) {                                                 //If there is serial data available
    received_byte = Serial.read();                                                //Reset the receive_counter variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();                  //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if (accelerometer_data_raw > 8200) {
    accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  }
  if (accelerometer_data_raw < -8200) {
    accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;
  }

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;        //Calculate the current angle according to the accelerometer

  if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5) {                  //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }

  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();                       //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();                     //Combine the two bytes to make one integer

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //  angle_gyro -= gyro_yaw_data_raw * 0.0000003;                          //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10) {
    pid_error_temp += pid_output * 0.015 ;
  }

  pid_i_mem += pid_i_gain * pid_error_temp;
  if (pid_i_mem > speed_m) {
    pid_i_mem = speed_m;
  }
  else if (pid_i_mem < -speed_m) {
    pid_i_mem = -speed_m;
  }
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > speed_m) {
    pid_output = speed_m;
  }
  else if (pid_output < -speed_m) {
    pid_output = -speed_m;
  }

  pid_last_d_error = pid_error_temp;

  if (pid_output < 5 && pid_output > -5) {
    pid_output = 0;
  }

  if (angle_gyro > 40 || angle_gyro < -40 || start == 0 || low_bat == 1) {
    pid_output = 0;
    pid_i_mem = 0;
    start = 0;
    self_balance_pid_setpoint = 0;
  }

  pid_output_left = pid_output;
  pid_output_right = pid_output;

//   if (received_byte == 82) {
//     pid_output_left += turning_speed;
//     pid_output_right -= turning_speed;
//   }
//   if (received_byte == 76) {
//     pid_output_left -= turning_speed;
//     pid_output_right += turning_speed;
//   }

//   if (received_byte == 70) {
//     if (pid_setpoint > -2.5) {
//       pid_setpoint -= pickup;
//     }
//     if (pid_output > max_target_speed * -1) {
//       pid_setpoint -= pickup;
//     }
//   }
//   if (received_byte == 66) {
//     if (pid_setpoint < 2.5) {
//       pid_setpoint += pickup;
//     }
//     if (pid_output < max_target_speed) {
//       pid_setpoint += pickup;
//     }
//   }
//   if (received_byte == 83) {
//     if (pid_setpoint > 0.5) {
//       pid_setpoint -= pickup;
//     }
//     else if (pid_setpoint < -0.5) {
//       pid_setpoint += pickup;
//     }
//     else pid_setpoint = 0;
//   }

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
  }
  else if (pid_output_left < 0) {
    pid_output_left = -speed_m - (1 / (pid_output_left - 9)) * 5500;
  }

  if (pid_output_right > 0) {
    pid_output_right = speed_m - (1 / (pid_output_right + 9)) * 5500;
  }
  else if (pid_output_right < 0) {
    pid_output_right = -speed_m - (1 / (pid_output_right - 9)) * 5500;
  }

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if (pid_output_left > 0) {
    left_motor = speed_m - pid_output_left;
  }
  else if (pid_output_left < 0) {
    left_motor = -speed_m - pid_output_left;
  }
  else {
    left_motor = 0;
  }

  if (pid_output_right > 0) {
    right_motor = speed_m - pid_output_right;
  }
  else if (pid_output_right < 0) {
    right_motor = -speed_m - pid_output_right;
  }
  else {
    right_motor = 0;
  }
  //Serial.println(left_motor);
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
  while (loop_timer > micros());
  loop_timer += 4000;
}


ISR(TIMER2_COMPA_vect) {
  //Left motor pulse calculations
  throttle_counter_left_motor ++;
  if (throttle_counter_left_motor > throttle_left_motor_memory) {
    throttle_counter_left_motor = 0;
    throttle_left_motor_memory = throttle_left_motor;
    if (throttle_left_motor_memory < 0) {
      PORTD &= 0b10111111;
      throttle_left_motor_memory *= -1;
    }
    else PORTD |= 0b01000000;
  }
  else if (throttle_counter_left_motor == 1) {
    PORTD |= 0b00001000;
  }
  else if (throttle_counter_left_motor == 2) {
    PORTD &= 0b11110111;
  }

  //right motor pulse calculations
  throttle_counter_right_motor ++;
  if (throttle_counter_right_motor > throttle_right_motor_memory) {
    throttle_counter_right_motor = 0;
    throttle_right_motor_memory = throttle_right_motor;
    if (throttle_right_motor_memory < 0) {
      PORTD |= 0b00100000;
      throttle_right_motor_memory *= -1;
    }
    else PORTD &= 0b11011111;
  }
  else if (throttle_counter_right_motor == 1) {
    PORTD |= 0b00000100;           //Set output 2 high to create a pulse for the stepper controller
  }
  else if (throttle_counter_right_motor == 2) {
    PORTD &= 0b11111011;
  }
}