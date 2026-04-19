#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "robot_types.h"
#include "sensors.h"
#include "motors.h"

//interval du loop 
constexpr int interval = 50;

// Position update from encoders + gyro
void get_position(Robot_status &robot_status,
                  const Robot_parameters &robot_parameters);

// Distance + heading control
double update_control_distance(double target_x,
                               double target_y,
                               Robot_status &robot_status,
                               const Robot_parameters &robot_parameters);
                               

// Angle-only control
double update_control_angle(double target_angle,
                            Robot_status &robot_status,
                            const Robot_parameters &robot_parameters);
                            

// Full robot state update (gyro + encoders + position)
void update_full_robot(Robot_status &robot_status, const Robot_parameters &robot_parameters);

// High-level movement
void go_to(double target_x, double target_y, double target_angle, Robot_status &robot_status, const Robot_parameters &robot_parameters, bool useShortestPath = true);

void total_reset_on_start(Motor_encoder &motorA, Motor_encoder &motorB, Robot_status &robot_status);

#endif