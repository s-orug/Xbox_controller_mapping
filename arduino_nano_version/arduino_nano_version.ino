#include <Wire.h>                                            

int gyro_address = 0x68;                                     
int acc_calibration_value = -200;                            

float pid_p_gain = 5;                                       
float pid_i_gain = 1;                                      
float pid_d_gain = 20;                                       
float turning_speed = 30;                                    
float max_target_speed = 10; 

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

void setup(){
  Serial.begin(9600);                                                       
  Wire.begin();                                                             
  TWBR = 12;                                                                

  TCCR2A = 0;                                                               
  TCCR2B = 0;                                                               
  TIMSK2 |= (1 << OCIE2A);                                                  
  TCCR2B |= (1 << CS21);                                                    
  OCR2A = 39;                                                               
  TCCR2A |= (1 << WGM21);                                                   

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
  Wire.beginTransmission(gyro_address);                                     
  Wire.write(0x1A);                                                         
  Wire.write(0x03);                                                         
  Wire.endTransmission();                                                   

  pinMode(2, OUTPUT);                                                       
  pinMode(3, OUTPUT);                                                       
  pinMode(4, OUTPUT);                                                       
  pinMode(5, OUTPUT);                                                       
  pinMode(13, OUTPUT);                                                      

  for(receive_counter = 0; receive_counter < 500; receive_counter++){       
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        
    Wire.beginTransmission(gyro_address);                                   
    Wire.write(0x43);                                                       
    Wire.endTransmission();                                                 
    Wire.requestFrom(gyro_address, 4);                                      
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             
    delayMicroseconds(3700);                                                
  }
  gyro_pitch_calibration_value /= 500;                                      
  gyro_yaw_calibration_value /= 500;                                        

  loop_timer = micros() + 4000;                                             

}

void loop(){
  if(Serial.available()){                                                   
    received_byte = Serial.read();                                          
    receive_counter = 0;                                                    
  }
  if(receive_counter <= 25)receive_counter ++;                              
  else received_byte = 0x00;                                                

  battery_voltage = (analogRead(0) * 1.222) + 85;

  if(battery_voltage < 1050 && battery_voltage > 800){                      
    digitalWrite(13, HIGH);                                                 
    low_bat = 1;                                                            
  }

  Wire.beginTransmission(gyro_address);                                     
  Wire.write(0x3F);                                                         
  Wire.endTransmission();                                                   
  Wire.requestFrom(gyro_address, 2);                                        
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      
  accelerometer_data_raw += acc_calibration_value;                          
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     
    angle_gyro = angle_acc;                                                 
    start = 1;                                                              
  }

  Wire.beginTransmission(gyro_address);                                     
  Wire.write(0x43);                                                         
  Wire.endTransmission();                                                   
  Wire.requestFrom(gyro_address, 4);                                        
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    

  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 
  if(pid_i_mem > 400)pid_i_mem = 400;                                       
  else if(pid_i_mem < -400)pid_i_mem = -400;

  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    
    pid_output = 0;                                                         
    pid_i_mem = 0;                                                          
    start = 0;                                                              
    self_balance_pid_setpoint = 0;                                          
  }

  pid_output_left = pid_output;                                             
  pid_output_right = pid_output;                                            

  if(received_byte & B00000001){                                            
    pid_output_left += turning_speed;                                       
    pid_output_right -= turning_speed;                                      
  }
  if(received_byte & B00000010){                                            
    pid_output_left -= turning_speed;                                       
    pid_output_right += turning_speed;                                      
  }

  if(received_byte & B00000100){                                            
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            
  }
  if(received_byte & B00001000){                                            
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 
  }   

  if(!(received_byte & B00001100)){                                         
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        
    else pid_setpoint = 0;                                                  
  }

  if(pid_setpoint == 0){                                                    
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  
  }

  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  while(loop_timer > micros());
  loop_timer += 4000;
}

ISR(TIMER2_COMPA_vect){

  throttle_counter_left_motor++;                                           
  if(throttle_counter_left_motor > throttle_left_motor_memory){             
    throttle_counter_left_motor = 0;                                        
    throttle_left_motor_memory = throttle_left_motor;                       
    if(throttle_left_motor_memory < 0){                                     
      PORTD &= 0b11110111;                                                  
      throttle_left_motor_memory *= -1;                                     
    }
    else PORTD |= 0b00001000;                                               
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             
  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             

  throttle_counter_right_motor++;                                          
  if(throttle_counter_right_motor > throttle_right_motor_memory){           
    throttle_counter_right_motor = 0;                                       
    throttle_right_motor_memory = throttle_right_motor;                     
    if(throttle_right_motor_memory < 0){                                    
      PORTD |= 0b00100000;                                                  
      throttle_right_motor_memory *= -1;                                    
    }
    else PORTD &= 0b11011111;
  }
  else if(throttle_counter_right_motor == 1)PORTD |= 0b00010000;
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11101111;  
}
