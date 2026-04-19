#ifndef ENCODERS_H
#define ENCODERS_H

#include "robot_types.h"

void readEncoder(Motor_encoder &motor);
void get_safely_encoder_pulses(Robot_status &robot_status,
                               Motor_encoder &motorA,
                               Motor_encoder &motorB);

#endif