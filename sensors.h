#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Normalize angle between -PI and PI
double normalize_angle(double angle);

// Read gyro angle (in degrees from sensor)
double get_gyro_angle(Adafruit_BNO055 &bno);

#endif