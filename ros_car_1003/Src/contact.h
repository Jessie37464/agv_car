//#include "pid.h"
#include "encoder.h"

#include "math.h"
#include <stdio.h>
#include "cstring"


void LeftMovingSpeedW(float val);//左轮方向和速度控制函数
void RightMovingSpeedW( float val2);//右轮方向和速度控制函数

void car_control(float rightspeed,float leftspeed);//小车速度转化和控制函数
