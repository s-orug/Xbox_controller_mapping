#ifndef MOTOR_H
#define MOTOR_H

#include "Pin.h"

class Motor {
private:
	int delay; //Delay between switching steps
	int direction; //0 = still, 1 = forward, -1 = backward
	int turning; //0 = not turning, 1 = clockwise/right, -1 = counterclockwise/left 

	// Pin *controlPin1;
	// Pin *controlPin2;
	// Pin *turningPin; //1 for normal, 0 when turning
    int controlPin1;
    int controlPin2;
    int turningPin;

public:
	Motor(int controlPin1, int controlPin2, int turningPin);
	~Motor();
	int getDelay();
	int getDirection();
	int getTurningDirection();
	void setDelay(int delay);
	void setDirection(int direction);
	void setTurningDirection(int turning);
	void runMotor(int time); //in microseconds
};

#endif