#include "robot_types.h"
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "sensors.h"
#include <ServoEasing.hpp>


//section servo 


void ServoSweep(ServoEasing &servo)
{
  const int MID_POS = 90;
  const int MIN_POS = 0;
  const int MAX_POS = 180;
  const int half_delay = 125;
  servo.setEasingType(EASE_CUBIC_IN_OUT);

  for (int i = 0; i < 3; i++) {
    // Move to minimum (blocking)
    servo.easeTo(MIN_POS, half_delay*2);
    delay(250);
    // Move to maximum (blocking)
    servo.easeTo(MAX_POS, half_delay*2);
    delay(250);
  }
    
}


//variables globales

unsigned long lastTime = 0;
//linterval est dans contrl.h

const int Start_button = A7;
const int Open_jaw_button = A6;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

const int LED_pin = A3;

Motor_encoder motorA {2 ,4, LOW, 0, 1}; //la variable direction flip assure que l'encodeur ajout des compte positif dans la direction positive
Motor_encoder motorB {3 ,7, LOW, 0, -1};

Robot_parameters robot_parameters {0.033, 0.2, 0.8, 0.15};

Robot_status robot_status {0.0, 0.0, 0.0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};

Motor_control motor_controlA {5, 8, 11}; // Motor A connections
Motor_control motor_controlB {6, 12, 13};//Motor B connections
//Standby, doit être HIGH pour que les moteurs fonctionnent
const int stby = A0;

//PID control variables, kp, ka, ki, kd
Pid pid {3, 0.3, 0.0, 0.0 };

ServoEasing servo_queue;
ServoEasing servo_cou;
ServoEasing servo_machoire;

const int machoire_open_angle = 105;
const int machoire_close_angle = 190;

const int cou_open_angle = 45;
const int cou_close_angle = 90;

const int interval_servo = 1000;


bool is_machoire_open = false;


// Interrupt Wrappers, a ajouter pour chaques encodeurs requis
void ISR_motorA() {
  readEncoder(motorA);
}
void ISR_motorB() {
  readEncoder(motorB);
}


 void setup() {
  //temporaire pour debug 
  Serial.begin(115200);
  Serial.println("full controle"); Serial.println("");

  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, LOW);
  // put your setup code here, to run once:
    // Set the encoder pins as inputs
  pinMode(motorA.pinA, INPUT_PULLUP); //verifier si on as besoin du pullup
  pinMode(motorA.pinB, INPUT_PULLUP);
  motorA.lastAState = digitalRead(motorA.pinA);

  pinMode(motorB.pinA, INPUT_PULLUP); //verifier si on as besoin du pullup
  pinMode(motorB.pinB, INPUT_PULLUP);
  motorB.lastAState = digitalRead(motorB.pinA);
  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(motorA.pinA), ISR_motorA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorB.pinA), ISR_motorB, CHANGE);

  // Set all the motor control pins to outputs
  pinMode(motor_controlA.pwm, OUTPUT);
  pinMode(motor_controlA.in1, OUTPUT);
  pinMode(motor_controlA.in2, OUTPUT);
  pinMode(motor_controlB.pwm, OUTPUT);
  pinMode(motor_controlB.in1, OUTPUT);
  pinMode(motor_controlB.in2, OUTPUT);

  pinMode(stby, OUTPUT);

  // Turn off motors - Initial state
  stop_motors(motor_controlA, motor_controlB, robot_status);
  // this lets the motor start when needed
  digitalWrite(stby, HIGH);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    while(1){
      digitalWrite(LED_pin, HIGH);
      delay(200);
      digitalWrite(LED_pin, LOW);
      delay(200);
    }
  }

  //Parametres des servos
    // Attach servo (pin, start angle)
  servo_machoire.attach(9, machoire_close_angle);
  servo_cou.attach(10, cou_close_angle+35);
  servo_queue.attach(A1, 180);

  // Set easing type (accel + decel)
  servo_machoire.setEasingType(EASE_CUBIC_IN_OUT);
  servo_cou.setEasingType(EASE_CUBIC_IN_OUT);
  servo_queue.setEasingType(EASE_CUBIC_IN_OUT);

  //attend que les servos atteignent la position de depart
  unsigned long currentTime = millis();
  while (millis() - currentTime < 1000);
    
  bno.setExtCrystalUse(true);
  delay(1500);  // let fusion settle
}

/*utiliser ceci pour non blocking servo motion:
servo.startEaseTo(...);  // starts motion
servo.update();          // progresses motion*/


void loop() {

  if(analogRead(Start_button) < 50){ //valeur donne 1023 si le bouton n<est pas presse et 0 si le bouton est presse
    //petit delay pour avoir un départ securitaire
    //Calculs pour le trajet court
    double distance_depotxy = 0.17; //avant 0.23335
    double target_x=0.57 + distance_depotxy + 0.03;
    double target_y=distance_depotxy;
    double target_angle_rel = atan(target_y/target_x);


    delay(500);
    //ici mettre la logique qui fait que langle 0 et les variables pertinentes se fait reset
    total_reset_on_start(motorA, motorB, robot_status);

    //s'oriente au début
    go_to(0, 0, target_angle_rel, robot_status, robot_parameters);

    target_angle_rel = 0.705;
    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters);

    //ici faire le dépot du poulet (attention le code est blockant)
    //ceci baisse le cou puis ouvre la bouche
    servo_cou.easeTo(cou_open_angle, interval_servo*2);
    delay(250);
    servo_machoire.easeTo(machoire_open_angle, interval_servo);

    target_x -= distance_depotxy - 0.03; 
    target_y -= distance_depotxy;
    target_angle_rel = 3*PI/4;

    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters);

        //ceci monte le cou puis ferme la bouche

    servo_machoire.easeTo(machoire_close_angle, interval_servo);
    servo_cou.easeTo(cou_close_angle+35, interval_servo);
    //clignoter le LED
    delay(500);
    for (int i = 0; i < 3; ++i) {
      digitalWrite(LED_pin, HIGH);
      delay(500);
      digitalWrite(LED_pin, LOW);
      delay(1000);
    }

    target_angle_rel = PI/2+0.1;

    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters);
    //mettre le reste de la logique de trajets

    target_y = 1.2888;
    target_angle_rel = -3*PI/4;

    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters);
    servo_cou.easeTo(cou_close_angle-35, interval_servo);
    //faire bouger la queue
    ServoSweep(servo_queue);

    //parcours optionnel
    //delais de 25s **oublie pas de le mettre lors de la compétition
    delay(25000);
    //s'alligne vers loptionnelle
    target_angle_rel = PI;
    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters);

    target_x = -0.03;
    target_y -= 0.05;

    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters);

    //tours sur lui mm

    target_angle_rel = -PI/2;

    go_to(target_x, target_y, target_angle_rel, robot_status, robot_parameters, false);
    servo_cou.easeTo(cou_close_angle+40, interval_servo);
    servo_queue.easeTo(90, interval_servo);
  }


  if(analogRead(Open_jaw_button) < 50){ //valeur donne 1023 si le bouton n<est pas presse et 0 si le bouton est presse
    //mettre la logique ici
    if(is_machoire_open){
      servo_machoire.easeTo(machoire_close_angle, interval_servo); //ceci ferme la bouche
    }
    else{
      servo_machoire.easeTo(machoire_open_angle, interval_servo);  //ceci ouvre la bouche et monte le cou
    }
    is_machoire_open = !is_machoire_open;
    delay(500);
  }

}
