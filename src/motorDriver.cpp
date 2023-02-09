/**
 * ESP32 Motor Library
 * 
 * Functions to set motor speed
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

#include "Arduino.h"
#include "motorDriver.h"


static uint8_t inA1 = 16;           /* Pin for Motor 0 */
static uint8_t inA2 = 17;           /* Pin for Motor 0 */
static uint8_t inB1 = 33;           /* Pin for Motor 1 */
static uint8_t inB2 = 19;           /* Pin for Motor 1 */
static int freq = 20000;            /* PWM Frequency */
static uint8_t Channel1 = 0;        /* PWM Channel for Motor 0 */
static uint8_t Channel2 = 1;        /* PWM Channel for Motor 1 */
static uint8_t motorChannel1 = 5;   /* PWM Pin for Motor 0 */
static uint8_t motorChannel2 = 18;  /* PWM Pin for Motor 0 */
static uint8_t resolution = 8;      /* PWM Resolution */


mclass::mclass() {
}

void mclass::SETUP() {
  pinMode(inA1, OUTPUT);            /* Initializing pins as Outputs */
  pinMode(inA2, OUTPUT);            /* Initializing pins as Outputs */
  pinMode(inB1, OUTPUT);            /* Initializing pins as Outputs */
  pinMode(inB2, OUTPUT);            /* Initializing pins as Outputs */

  // initialize pwm channels
  ledcSetup(Channel1, freq, resolution);    
  ledcAttachPin(motorChannel1, Channel1);   

  ledcSetup(Channel2, freq, resolution);    
  ledcAttachPin(motorChannel2, Channel2);   
}

void mclass::motor_direction(Motors motor_ch, Direction dir) {
  // parameters: motor_ch, dir, speed 
  // motor_ch: 0 for motor 0, 1 for motor 1
  // dir: 0 for forward, 1 for backward
  // speed: 0 to 255
  // returns: none

  if (motor_ch == 0)
  {
    if (dir == Forward) // set motor direction to forward or backward depending on the value of dir for motor 0
    {
      digitalWrite(inA1,HIGH);  
      digitalWrite(inA2,LOW);   
    }
    else 
    {
      digitalWrite(inA1,LOW);   
      digitalWrite(inA2,HIGH);  
    }
  }

  else
  {
    if (dir == Forward) // set motor direction to forward or backward depending on the value of dir for motor 1
    {
      digitalWrite(inB1,LOW);   
      digitalWrite(inB2,HIGH);  
    }
    else
    {
      digitalWrite(inB1,HIGH);  
      digitalWrite(inB2,LOW);   
    }
  }
}

void mclass::set_speed(Motors motor_ch, Direction dir, int new_speed) {
  // parameters: motor_ch, dir, speed
  // motor_ch: 0 for motor 0, 1 for motor 1
  // dir: 0 for forward, 1 for backward
  // speed: 0 to 255
  // returns: none

  motor_direction(motor_ch, dir);   /* polarize the motors before setting the speed */
  
  if (new_speed < 0)              /* set the speed to 0 if it is less than 0 */ 
  {
    new_speed = 0;
  }

  else if (new_speed > 255)     /* set the speed to 255 if it is greater than 255 */
  {
    new_speed = 255;
  }

  else if (new_speed >= 0 && new_speed <= 255) /* check if the speed is within the range and set it to the new speed if it is within the range */
  {
    new_speed = new_speed;
  }

  if (motor_ch == 0)            /* write the speed to the first motor */
  {
    ledcWrite(Channel1, new_speed);
  }
  else if (motor_ch == 1)       /* write the speed to the second motor */
  {
    ledcWrite(Channel2, new_speed);
  }
}

mclass motorobject = mclass();  /* creating an object of class motor */
