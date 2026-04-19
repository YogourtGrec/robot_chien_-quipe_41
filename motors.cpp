#include "motors.h"

extern const int stby;

void stop_motor(Motor_control &motor_control){
  digitalWrite(motor_control.pwm, LOW);
  digitalWrite(motor_control.in1, LOW);
  digitalWrite(motor_control.in2, LOW);
}

void stop_motors(Motor_control &motor_controlA, Motor_control &motor_controlB, Robot_status &robot_status){
  robot_status.speedA = 0.0;
  robot_status.speedB = 0.0;
  stop_motor(motor_controlA);
  stop_motor(motor_controlB);
}

void set_motor_speed(double set_speed, Motor_control &motor_control, bool cw)
{
  const int MIN_PWM = 40;   // augmente un peu pour être sûr
  const double MAX_SPEED = 0.4; // ⚠️ A AJUSTER

  int pwm_value = 0;

  if (abs(set_speed) > 0)
  {
    pwm_value = MIN_PWM + (abs(set_speed) / MAX_SPEED) * (255 - MIN_PWM);
  }

  // clamp
  if (pwm_value > 255)
    pwm_value = 255;

  // direction
  bool direction = cw;
  if (set_speed < 0)
  {
    direction = !direction;
  }

  if (direction)
  {
    digitalWrite(motor_control.in1, HIGH);
    digitalWrite(motor_control.in2, LOW);
  }
  else
  {
    digitalWrite(motor_control.in1, LOW);
    digitalWrite(motor_control.in2, HIGH);
  }

  analogWrite(motor_control.pwm, pwm_value);
}
