#include "control.h"

//external variables here

extern unsigned long lastTime;
extern Motor_control motor_controlA;
extern Motor_control motor_controlB;

extern Motor_encoder motorA;
extern Motor_encoder motorB;

extern Adafruit_BNO055 bno;

extern Pid pid;


void get_position(Robot_status &robot_status, const Robot_parameters &robot_parameters){
  double dA = (robot_status.encoder_pulsesA - robot_status.last_encoder_pulsesA) * robot_parameters.distance_per_pulse;
  double dB = (robot_status.encoder_pulsesB - robot_status.last_encoder_pulsesB) * robot_parameters.distance_per_pulse;

  double dCenter = (dA + dB)/2;

  double angle = robot_status.gyro_angle;

  robot_status.x += dCenter * cos(angle);
  robot_status.y += dCenter * sin(angle);
}



double update_control_distance(double target_x, double target_y, Robot_status &robot_status, const Robot_parameters &robot_parameters){
  double dx = target_x - robot_status.x;
  double dy = target_y - robot_status.y;

  double distanceError = sqrt(dx*dx + dy*dy);

  double targetHeading = atan2(dy, dx);
  double angleError = normalize_angle(targetHeading - robot_status.gyro_angle);

  bool reverse = false;

  // allow backward motion
  if (fabs(angleError) > 3*PI/4) {
    Serial.println("stuff is happening : ");
    targetHeading = normalize_angle(targetHeading + PI); // flip direction
    angleError = normalize_angle(targetHeading - robot_status.gyro_angle);
    reverse = true;
  }
  
  //double integral = 0; //mettre la logique plus tard
  //double derivative = (error - prev_error) / dt;
  double linearSpeed = pid.Kp * distanceError; //+ Ki * integral + Kd * derivative;
  if (reverse) {
  linearSpeed = -linearSpeed;  // THIS is the key
  }

  linearSpeed *= cos(angleError); //empeche le robot de vrm avancer si langle est vrm pas bon
  double angular_speed = pid.Ka * angleError;
  double next_speedA = linearSpeed - angular_speed;
  double next_speedB = linearSpeed + angular_speed;

  // --- ADD THIS BLOCK HERE ---
  /*double dt = interval / 1000.0;

  double measuredSpeedA = (robot_status.encoder_pulsesA - robot_status.last_encoder_pulsesA) / dt;
  double measuredSpeedB = (robot_status.encoder_pulsesB - robot_status.last_encoder_pulsesB) / dt;

  double speedError = measuredSpeedA - measuredSpeedB;
  double correction = 0.01 * speedError;

  next_speedA -= correction;
  next_speedB += correction;*/
  // --- END BLOCK ---

  //logique de saturation ici
  //tourne sur lui meme si il nest pas du tout dans la bonne direction
  if(fabs(angleError) > 0.25*PI){ 
    next_speedA = - angular_speed;
    next_speedB = angular_speed;
  }
  //limiteur dacceleration
  double max_delta = robot_parameters.max_acceleration * (interval / 1000.0);

  double deltaA = next_speedA - robot_status.speedA;
  double deltaB = next_speedB - robot_status.speedB;

  if (fabs(deltaA) > max_delta)
    next_speedA = robot_status.speedA + copysign(max_delta, deltaA);

  if (fabs(deltaB) > max_delta)
    next_speedB = robot_status.speedB + copysign(max_delta, deltaB);

  // Save speeds
  robot_status.speedA = next_speedA;
  robot_status.speedB = next_speedB;

  // Send to motors
  set_motor_speed(next_speedA, motor_controlA, false);
  set_motor_speed(next_speedB, motor_controlB, true);
  return(distanceError);
}



double update_control_angle(double target_angle, Robot_status &robot_status, const Robot_parameters &robot_parameters, bool useShortestPath){
    // Compute angle error
    double angleError;

    if (useShortestPath)
      angleError = normalize_angle(target_angle - robot_status.gyro_angle);
    else
      angleError = target_angle - robot_status.gyro_angle;

    // PID derivative
    static double lastError = angleError;
    double dt = interval / 1000.0;
    double derivative = (angleError - lastError) / dt;
    lastError = angleError;

    // PID output for angular speed
    double pid_output = pid.Ka * angleError - pid.Kd * derivative;

    // Limit change per loop (acceleration)
    double max_delta = robot_parameters.max_acceleration_rotation * dt;
    double delta = pid_output - robot_status.speedA;
    if (fabs(delta) > max_delta)
        pid_output = robot_status.speedA + copysign(max_delta, delta);

    // Deceleration-aware limit: ensure we can stop without overshooting
    double stopping_speed = sqrt(2.0 * robot_parameters.max_acceleration_rotation * fabs(angleError)* 0.5);
    if (fabs(pid_output) > stopping_speed)
        pid_output = copysign(stopping_speed, pid_output);

    // Apply to motors
    double speedA = pid_output;
    double speedB = -pid_output;

    robot_status.speedA = speedA;
    robot_status.speedB = speedB;

    set_motor_speed(speedA, motor_controlA, true);
    set_motor_speed(speedB, motor_controlB, false);

    return angleError;
}


void update_full_robot(Robot_status &robot_status, const Robot_parameters &robot_parameters){

  double new_raw_angle = get_gyro_angle(bno) * PI / 180.0;
  double delta = normalize_angle(new_raw_angle - robot_status.gyro_last_angle);
  robot_status.gyro_angle += delta;       // accumulated angle from start
  robot_status.gyro_last_angle = new_raw_angle;
  //le petit bloc en haut est donne par chat et je ne le comprend pas, deroulage dangle???
  get_safely_encoder_pulses(robot_status, motorA, motorB);
  get_position(robot_status, robot_parameters);

  //print des trucs à des fins de debug
  Serial.println("###################################################");
  Serial.print("encoder A : ");
  Serial.println(robot_status.encoder_pulsesA);
  Serial.print("encoder B : ");
  Serial.println(robot_status.encoder_pulsesB);
  Serial.print("gyro angle : ");
  Serial.println(robot_status.gyro_angle);
  Serial.print("position x : ");
  Serial.println(robot_status.x);
  Serial.print("position y : ");
  Serial.println(robot_status.y);
  Serial.print("Speed A : ");
  Serial.println(robot_status.speedA);
  Serial.print("Speed B : ");
  Serial.println(robot_status.speedB);
}


//prend les parametre de positions absolue selon le depart
void go_to(double target_x, double target_y, double target_angle, Robot_status &robot_status, const Robot_parameters &robot_parameters, bool useShortestPath){
  update_full_robot(robot_status, robot_parameters);
  double distanceError = update_control_distance(target_x, target_y, robot_status, robot_parameters);
  double distance_tolerance = 0.03;
  unsigned long currentTime = millis();
  while(fabs(distanceError) > distance_tolerance) {
    currentTime = millis();
    if (currentTime - lastTime >= interval) {
      lastTime = currentTime; // Update the last execution time
      update_full_robot(robot_status, robot_parameters);
      distanceError = update_control_distance(target_x, target_y, robot_status, robot_parameters);
    }
  }
  stop_motors(motor_controlA, motor_controlB, robot_status);

  update_full_robot(robot_status, robot_parameters);
  double angleError = update_control_angle(target_angle, robot_status, robot_parameters, useShortestPath);
  double angle_tolerance = 0.05; //en radians 
  while(fabs(angleError) > angle_tolerance) {
    currentTime = millis();
    if (currentTime - lastTime >= interval) {
      Serial.print("angle error : ");
      Serial.println(angleError);
      lastTime = currentTime; // Update the last execution time
      update_full_robot(robot_status, robot_parameters);
      angleError = update_control_angle(target_angle, robot_status, robot_parameters, useShortestPath);
    }
  }
  stop_motors(motor_controlA, motor_controlB, robot_status);
}


void total_reset_on_start(Motor_encoder &motorA, Motor_encoder &motorB, Robot_status &robot_status){
  //reset les encodeurs
  noInterrupts();
  motorA.pulses = 0;
  motorB.pulses = 0;
  interrupts();
  //reset les positions des robots
  robot_status.encoder_pulsesA = 0.0;
  robot_status.encoder_pulsesB = 0.0;
  robot_status.last_encoder_pulsesA = 0.0;
  robot_status.last_encoder_pulsesB = 0.0;
  robot_status.x = 0.0;
  robot_status.y = 0.0;

  // read gyro at start
  double start_angle = normalize_angle(get_gyro_angle(bno) * PI / 180.0);

  robot_status.gyro_angle = 0.0;        // set the accumulated angle to 0
  robot_status.gyro_last_angle = start_angle; // store the raw gyro value for delta
}