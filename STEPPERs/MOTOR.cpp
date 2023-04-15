#include <wiringPi.h>
#include <unistd.h>
#include "Motor.h"

Motor::Motor(int controlPin1, int controlPin2, int turningPin){
	wiringPiSetupGpio();  //Sets up using GPIO pin numbers
	this->delay = 2500;
	this->direction = 0;
	this->turning = 0;
	this->controlPin1 = new Pin(controlPin1);
	this->controlPin2 = new Pin(controlPin2);
	this->turningPin = new Pin(turningPin);
	this->controlPin1->setState(1);
	this->controlPin2->setState(1);
	this->turningPin->setState(1);
	this->controlPin1->setOutput(0);
	this->controlPin2->setOutput(0);
	this->turningPin->setOutput(1);
}

Motor::~Motor(){
	this->controlPin1->setOutput(0);
	this->controlPin2->setOutput(0);
	this->turningPin->setOutput(0);
	delete this->controlPin1;
	delete this->controlPin2;
	delete this->turningPin;
}

int Motor::getDelay(){
	return this->delay;
}

int Motor::getDirection(){
	return this->direction;
}

int Motor::getTurningDirection(){
	return this->turning;
}

void Motor::setDelay(int delay){
	this->delay = delay;
}

void Motor::setDirection(int direction){
	this->direction = direction;
}

void Motor::setTurningDirection(int turning){
	this->turning = turning;
	if(this->turning == 1 || this->turning == -1){
		this->turningPin->setOutput(0);
	} else {
		this->turningPin->setOutput(1);
	}
}

void Motor::runMotor(int time){
	int wait = time;
	if(this->direction == 1){
		while(wait > 0){
			this->controlPin1->setOutput(0);
			this->controlPin2->setOutput(0);
			usleep(this->delay);
			this->controlPin1->setOutput(0);
			this->controlPin2->setOutput(1);
			usleep(this->delay);
			this->controlPin1->setOutput(1);
			this->controlPin2->setOutput(0);
			usleep(this->delay);
			this->controlPin1->setOutput(1);
			this->controlPin2->setOutput(1);
			usleep(this->delay);
			wait--;
		} 
	} else if(this->direction == -1){
		while(wait > 0){
			this->controlPin1->setOutput(1);
			this->controlPin2->setOutput(1);
			usleep(this->delay);
			this->controlPin1->setOutput(1);
			this->controlPin2->setOutput(0);
			usleep(this->delay);
			this->controlPin1->setOutput(0);
			this->controlPin2->setOutput(1);
			usleep(this->delay);
			this->controlPin1->setOutput(0);
			this->controlPin2->setOutput(0);
			usleep(this->delay);
			wait--;
		} 
	} else {
		if(this->turning == 1){
			while(wait > 0){
				this->controlPin1->setOutput(0);
				this->controlPin2->setOutput(0);
				usleep(this->delay);
				this->controlPin1->setOutput(0);
				this->controlPin2->setOutput(1);
				usleep(this->delay);
				this->controlPin1->setOutput(1);
				this->controlPin2->setOutput(0);
				usleep(this->delay);
				this->controlPin1->setOutput(1);
				this->controlPin2->setOutput(1);
				usleep(this->delay);
				wait--;
			} 
		} else if(this->turning == -1){
			while(wait > 0){
				this->controlPin1->setOutput(1);
				this->controlPin2->setOutput(1);
				usleep(this->delay);
				this->controlPin1->setOutput(1);
				this->controlPin2->setOutput(0);
				usleep(this->delay);
				this->controlPin1->setOutput(0);
				this->controlPin2->setOutput(1);
				usleep(this->delay);
				this->controlPin1->setOutput(0);
				this->controlPin2->setOutput(0);
				usleep(this->delay);
				wait--;
			} 
		}
	}
}