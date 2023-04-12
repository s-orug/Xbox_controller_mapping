CC=g++
CFLAGS = -Wall -g


IMU=mpu6050
CONTROLLER=xbox
STEPPER=stepper
TEST=test
STEPPERS=steppers
BETA=beta

main: 
	$(CC) $(CFLAGS) -o main main.cpp -lwiringPi

BETA:
	$(CC) $(CFLAGS) -o $(BETA) $(BETA).cpp -pthread -lwiringPi

IMU:
	$(CC) $(CFLAGS) -o $(IMU) $(IMU).cpp -pthread -lwiringPi

CONTROLLER:
	$(CC) $(CFLAGS) -o $(CONTROLLER) $(CONTROLLER).cpp

STEPPER:
	$(CC) $(CFLAGS) -o $(STEPPER) $(STEPPER).cpp -lwiringPi

STEPPERS:
	$(CC) $(CFLAGS) -o $(STEPPERS) $(STEPPERS).cpp -pthread -lwiringPi

ALL:
	$(CC) $(CFLAGS) -o $(STEPPER) $(STEPPER).cpp -lwiringPi
	$(CC) $(CFLAGS) -o $(CONTROLLER) $(CONTROLLER).cpp
	$(CC) $(CFLAGS) -o $(IMU) $(IMU).cpp -lwiringPi
	$(CC) $(CFLAGS) -o main main.cpp -lwiringPi

test:
	$(CC) $(CFLAGS) -o $(TEST) $(TEST).cpp -pthread

clean:
	rm -rf $(IMU) $(CONTROLLER) $(STEPPER) $(BETA) $(TEST) $(STEPPERS) main *.o
