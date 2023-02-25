CC=g++
CFLAGS = -Wall -g

IMU=mpu6050
CONTROLLER=xbox


main: $(IMU) $(CONTROLLER)

IMU:
	$(CC) $(CFLAGS) -o $(IMU) $(IMU).cpp -lwiringPi

CONTROLLER:
	$(CC) $(CFLAGS) -o $(CONTROLLER) $(CONTROLLER).cpp

clean:
	rm -rf $(IMU) $(CONTROLLER) mpu6050 xbox *.o