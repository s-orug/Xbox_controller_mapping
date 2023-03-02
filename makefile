CC=g++
CFLAGS = -Wall -g

IMU=mpu6050
CONTROLLER=xbox
STEPPER=stepper

main: $(IMU) $(CONTROLLER)

IMU:
	$(CC) $(CFLAGS) -o $(IMU) $(IMU).cpp -lwiringPi

CONTROLLER:
	$(CC) $(CFLAGS) -o $(CONTROLLER) $(CONTROLLER).cpp

STEPPER:
	$(CC) $(CFLAGS) -o $(STEPPER) $(STEPPER).cpp

clean:
	rm -rf $(IMU) $(CONTROLLER) $(STEPPER) mpu6050 xbox stepper *.o