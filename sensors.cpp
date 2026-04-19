#include "sensors.h"

//retourne langle de pi a -pi 
double normalize_angle(double angle){
  return atan2(sin(angle), cos(angle));
}


double get_gyro_angle(Adafruit_BNO055 &bno){
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  return(360.0 - event.orientation.x); //360 - angle car le senseur est à l'envere
}