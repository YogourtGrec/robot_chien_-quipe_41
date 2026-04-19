#ifndef MOTORS_H
#define MOTORS_H

#include "robot_types.h"
#include "encoders.h"


void set_motor_speed(double set_speed, Motor_control &motor_control, bool cw);
void stop_motor(Motor_control &motor_control);
void stop_motors(Motor_control &A, Motor_control &B, Robot_status &status);

#endif