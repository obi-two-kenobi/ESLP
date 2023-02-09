#include <Arduino.h>
#include "motorDriver.h"
#define LED_BOARD 2 //change here the pin of the board to V2
#include <OPT3101.h>
#include <Wire.h>
#include "AWS.h"
#include <thread>
#include <mutex>

// Backward and Forward define the direction of the motors and they are used in reversed manner
// Forward is used to move the rover backwards and Backward is used to move the rover forward


// amplitude is an array of the amplitudes from the left, front and right sensors
uint16_t amplitudes[3];
// distances is an array of the distances from the left, front and right sensors
int16_t distances[3]={0,0,0};
// sensor is the OPT3101 sensor object
OPT3101 sensor;

// mytarget is the target location of the rover
struct coordinate mytarget;
// roverLoc struct is used to store the location of the rover
struct coordinatesWithAngle roverLoc;

// correctionAngle is the angle to turn the rover to be in the right direction
int correctionAngle;
// LinearDistance is the distance to move the rover to be in the right location
int LinearDistance;

void getFromAWS(void* parameter) {
  // Parameters: void* parameter - needed for xTaskCreate function
  // Return: void
  // run the stayConnected function of the AWS class to keep the connection alive and to get the data from the AWS server 

  while (true)
  {
    awsobject.stayConnected();
    delay(20);
  }
}

void startSampling(OPT3101& sensor) {
  // Parameters: OPT3101& sensor - the sensor to be initialized
  // Return: void
  // Initialize the sensor
  // if the sensor is not initialized, print the error and stop the program
  // set the frame timing to 256
  // set the channel to 0
  // set the brightness to adaptive
  // start sampling
  
  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }
  sensor.setFrameTiming(256);
  sensor.setChannel(0);
  sensor.setBrightness(OPT3101Brightness::Adaptive);
  sensor.startSample();
}

void checkObstaclesUsingSensor(int16_t* distances) {
  // Parameters: double* distances - the distances read from the sensors
  // Return: void
  // if the distance from the left sensor is less than 50 and greater than 0, move the rover backwards and turn right then move forward
  // if the distance from the right sensor is less than 50 and greater than 0, move the rover backwards and turn left then move forward
  if(distances[0] < 50 && distances[0] > 0){
    motorobject.set_speed(MotorA, Forward, 120);
    motorobject.set_speed(MotorB, Forward, 120);
    delay(300);
    motorobject.set_speed(MotorA, Forward, 120);
    motorobject.set_speed(MotorB, Backward, 120);
    delay(300);
    motorobject.set_speed(MotorA, Backward, 120);
    motorobject.set_speed(MotorB, Backward, 120);
    delay(300);
  }else if(distances[2] < 50 && distances[2] > 0){
    motorobject.set_speed(MotorA, Forward, 120);
    motorobject.set_speed(MotorB, Forward, 120);
    delay(300);
    motorobject.set_speed(MotorA, Backward, 120);
    motorobject.set_speed(MotorB, Forward, 120);
    delay(300);
    motorobject.set_speed(MotorA, Backward, 120);
    motorobject.set_speed(MotorB, Backward, 120);
    delay(300);
  }
}

void moveRoverBasedOnInfoFromAWS(){
  // Parameters: void
  // Return: void
  // get the correction angle and the linear distance from the AWS server
  // if the correction angle is less than 5 and greater than -5, move the rover forward
  // if the correction angle is greater than 5, turn the rover right
  // if the correction angle is less than -5, turn the rover left
  // if the linear distance is less than 10, stop the rover
  // if the linear distance is greater than 10, move the rover forward

  // rotationSpeed is the speed of the motors to turn the rover and it is calculated based on the correction angle 
  // rotationSpeed is between 100 and 255
  auto rotationSpeed = (abs(correctionAngle)/180.0)*255;
  rotationSpeed = (rotationSpeed < 100) ? 100 : rotationSpeed;

  // movmentSpeed is the speed of the motors to move the rover  and it is calculated based on the linear distance
  // movmentSpeed is between 100 and 255
  auto movmentSpeed = (abs(LinearDistance)/100.0)*255;
  movmentSpeed = (movmentSpeed < 100) ? 100 : movmentSpeed;

  auto angleThreshold = 5;

  if (correctionAngle >= -1*angleThreshold && correctionAngle <= angleThreshold){
    if(LinearDistance > 10){
      motorobject.set_speed(MotorA, Backward, movmentSpeed);
      motorobject.set_speed(MotorB, Backward, movmentSpeed);
    }
    else{
      motorobject.set_speed(MotorA, Backward, 0);
      motorobject.set_speed(MotorB, Backward, 0);
    }
    delay(1);
  }
  else if(correctionAngle < -1*angleThreshold){
    motorobject.set_speed(MotorA, Forward, rotationSpeed);
    motorobject.set_speed(MotorB, Backward, rotationSpeed);
    delay(1);
  }
  else if(correctionAngle > angleThreshold){
    motorobject.set_speed(MotorA, Backward, rotationSpeed);
    motorobject.set_speed(MotorB, Forward, rotationSpeed);
    delay(1);
  }
  else{ 
    motorobject.set_speed(MotorA, Backward, 0);
    motorobject.set_speed(MotorB, Backward, 0);
    delay(1);
  }
}

void setup(){
  // set the LED pin as an output:
  pinMode(LED_BOARD, OUTPUT);
  // Initialize serial communication
  Serial.begin(115200);
  // Initialize the motor driver
  motorobject.SETUP();  
  // Wire is used to communicate with the OPT3101
  Wire.begin(27,26);
  //inisticate the OPT3101 and start sampling
  startSampling(sensor);
  //inisticate the AWS class
  awsobject.connectAWS();
  // Create a task to handle the AWS connection using the FreeRTOS task 
  xTaskCreate(getFromAWS, "getFromAWS", 10000, NULL, 1, NULL);
}

void loop(){
  // set the speed of the motors to 0 to stop the rover
  motorobject.set_speed(MotorA, Backward, 0);
  motorobject.set_speed(MotorB, Backward, 0);

  // if the sample is done, read the output registers
  if (sensor.isSampleDone())
    {
      sensor.readOutputRegs();
      // distances is an array of the distances from the left, front and right sensors
      distances[sensor.channelUsed] = sensor.distanceMillimeters;
      #ifdef DEBUG
      if (sensor.channelUsed == 2)
      {

        Serial.print("left:  " + String(distances[0]/10.0) + "cm, " );
        Serial.print("front: " + String(distances[1]/10.0) + "cm, " );
        Serial.println("right: " + String(distances[2]/10.0) + "cm " ); 
      }
      #endif
      // change the channel to the next channel and start sampling again
      sensor.nextChannel();
      sensor.startSample();
    }

  checkObstaclesUsingSensor(distances);
  moveRoverBasedOnInfoFromAWS();
  delay(10);
}

