#include <wiringPi.h>
#include <iostream>       
#include <thread>         
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

long long int t_prev=0, t_now=0;
float dt;
int k = 0;
struct timeval start_time, end_time;
long milli_time, seconds, useconds;

void foo()
{
  while(true){
   gettimeofday(&start_time, NULL);
   seconds = start_time.tv_sec;
  }
}

int main()
{
    std::thread first(foo);     
    while(true){
      std::cout << seconds << std::endl;
      delayMicroseconds(1000000);
    }
    
    first.join();
    return 0;
}
