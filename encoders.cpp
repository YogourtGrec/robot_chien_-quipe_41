#include "encoders.h"

void get_safely_encoder_pulses(Robot_status &robot_status, Motor_encoder &motorA, Motor_encoder &motorB){
  long encoder_pulsesA;
  long encoder_pulsesB;
  noInterrupts();
  encoder_pulsesA = motorA.pulses;
  encoder_pulsesB = motorB.pulses;
  interrupts();
  robot_status.last_encoder_pulsesA = robot_status.encoder_pulsesA;
  robot_status.encoder_pulsesA = encoder_pulsesA * motorA.direction_flip;
  robot_status.last_encoder_pulsesB = robot_status.encoder_pulsesB;
  robot_status.encoder_pulsesB = encoder_pulsesB * motorB.direction_flip;
}

// Generic Encoder Update Function
void readEncoder(Motor_encoder &motor) {
  // Read the current state of Channel A
  int currentAState = digitalRead(motor.pinA);
  // Determine the direction of rotation
  if (currentAState != motor.lastAState) {
    if (digitalRead(motor.pinB) != currentAState) {
      motor.pulses++;
    } else {
      motor.pulses--;
    }
    motor.lastAState = currentAState;
  }
}