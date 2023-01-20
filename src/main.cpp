#include <Arduino.h>
#include "motorDriver.h"
#define LED_BOARD 2 //change here the pin of the board to V2
#include <OPT3101.h>
#include <Wire.h>
#include "AWS.h"

//not backwards and forwards is flipped

uint16_t amplitudes[3];
int16_t distances[3]={0,0,0};
OPT3101 sensor;

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


}

void loop(){
  motorobject.set_speed(MotorA, Backward, 0);
  motorobject.set_speed(MotorB, Backward, 0);
  awsobject.stayConnected();

  // if (sensor.isSampleDone())
  //   {
  //     sensor.readOutputRegs();

  //     amplitudes[sensor.channelUsed] = sensor.amplitude;
  //     distances[sensor.channelUsed] = sensor.distanceMillimeters;

  //     if (sensor.channelUsed == 2)
  //     {

  //         Serial.print("left:  " + String(distances[0]/10.0) + "cm, " );
  //         Serial.print("front: " + String(distances[1]/10.0) + "cm, " );
  //         Serial.println("left: " + String(distances[1]/10.0) + "cm " ); 
  //         //awsobject.publishMessage(distances); Won't publish anythoing to AWS       
  //     }
  //     sensor.nextChannel();
  //     sensor.startSample();
  //   }

  //awsobject.publishMessage(distances);
  //awsobject.publishMessage(distances);

  
   delay(10);
}

