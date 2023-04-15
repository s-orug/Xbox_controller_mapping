#include <wiringPi.h>
#include <cmath>
#include <iostream>

#define MIN_VELOCITY_THRESHOLD 1e-6
#define MAX_STEP_DELAY 1e6
#define STEP_PULSE_WIDTH 1e-6

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

int main() {
    wiringPiSetupGpio();
    Stepper motor(13, 6, 200);

    float vel = 1;
    motor.set_direction(1);
    motor.set_velocity(-10 * M_PI);

    while (true) {
        motor.loop();
	/*if (vel > -100)
	  {	vel = vel - 0.00001;
	motor.set_velocity(vel);
	*/std::cout << vel << std::endl;
    }

    return 0;
}
