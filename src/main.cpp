#include <Arduino.h>
#include "motorDriver.h"
#define LED_BOARD 2 //change here the pin of the board to V2
#include <OPT3101.h>
#include <Wire.h>
#include "AWS.h"
#include <thread>
#include <mutex>
//not backwards and forwards is flipped

uint16_t amplitudes[3];
int16_t distances[3]={0,0,0};
OPT3101 sensor;
struct target mytarger;
struct rover roverLoc;

int correctionAngle;
int LinearDistance;

void getFromAWS(void* parameter) {
  while (true)
  {
    awsobject.stayConnected();
    delay(20);
  }
}

void setup(){

  pinMode(LED_BOARD, OUTPUT);
  Serial.begin(115200);
  motorobject.SETUP();  
  
  Wire.begin(27,26);
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

  //inisticate the AWS class
  awsobject.connectAWS();

  // std::thread getFromAWS([&]{
  //   while (true)
  //   {
  //     awsobject.stayConnected();
  //     delay(100);
  //   }  
  // }); getFromAWS.detach();
  xTaskCreate(getFromAWS, "getFromAWS", 10000, NULL, 1, NULL);
}

void loop(){
  motorobject.set_speed(MotorA, Backward, 0);
  motorobject.set_speed(MotorB, Backward, 0);
  if (sensor.isSampleDone())
    {
      sensor.readOutputRegs();

      amplitudes[sensor.channelUsed] = sensor.amplitude; 
      distances[sensor.channelUsed] = sensor.distanceMillimeters;

      if (sensor.channelUsed == 2)
      {

          Serial.print("left:  " + String(distances[0]/10.0) + "cm, " );
          Serial.print("front: " + String(distances[1]/10.0) + "cm, " );
          Serial.println("left: " + String(distances[1]/10.0) + "cm " ); 
          //awsobject.publishMessage(distances); Won't publish anythoing to AWS       
      }
      sensor.nextChannel();
      sensor.startSample();
    }
    

  //awsobject.publishMessage(distances);
  //awsobject.publishMessage(distances);

  auto rotationSpeed = (abs(correctionAngle)/180.0)*255;
  rotationSpeed = (rotationSpeed < 70) ? 70 : rotationSpeed;

  auto movmentSpeed = (abs(LinearDistance)/100.0)*255;
  movmentSpeed = (movmentSpeed < 70) ? 70 : movmentSpeed;

  auto angleThreshold = 5;

  if (correctionAngle >= -1*angleThreshold && correctionAngle <= angleThreshold){
    // motorobject.set_speed(MotorA, Backward, 0);
    // motorobject.set_speed(MotorB, Backward, 0);
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
  
  delay(10);
}

