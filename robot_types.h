#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>


struct Motor_encoder
{
  const uint8_t pinA;
  const uint8_t pinB;

  volatile int lastAState;
  volatile long pulses;
  const int direction_flip;
};

struct Robot_status
{
  double gyro_angle;
  double gyro_last_angle;
  double gyro_start_angle;
  long encoder_pulsesA;
  long encoder_pulsesB;
  long last_encoder_pulsesA;
  long last_encoder_pulsesB;
  double x;
  double y;
  double speedA;
  double speedB;
};

struct Robot_parameters
{
  double wheel_radius_m;
  double wheelBase_m;
  double max_acceleration;
  double max_acceleration_rotation;
  double distance_per_pulse;
  double max_speed;


  Robot_parameters(double radius, double wheelbase, double max_accel, double max_accel_rot)
  {
    wheel_radius_m = radius;
    wheelBase_m = wheelbase;
    max_acceleration = max_accel;
    max_acceleration_rotation = max_accel_rot;
    distance_per_pulse = (2 * PI * wheel_radius_m) / (150.0 * 11.0 * 2); //le diviser par deux est expérimentale
    max_speed = (40.0 / 60.0) * 2 * PI * wheel_radius_m;
  }
};

struct Motor_control
{
  // Motor A connections
  const int pwm;
  const int in1;
  const int in2;
};

struct Pid
{
  const float Kp;
  const float Ka;
  const float Ki;
  const float Kd;
};



#endif