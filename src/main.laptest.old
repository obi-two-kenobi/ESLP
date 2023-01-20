#include <Arduino.h>
#include "motorDriver.h"
#define LED_BOARD 2 //change here the pin of the board to V2
#include <OPT3101.h>
#include <Wire.h>
#include "AWS.h"

//not backwards and forwards is flipped

uint16_t amplitudes[3];
int16_t distances[3];
OPT3101 sensor;

void motorTask(void* nth){
  while (true)
  {
    if(distances[1] < 100 && distances[0] < 100+20 && distances[2] < 100+20) {
      motorobject.set_speed(MotorA, Forward, 0);
      motorobject.set_speed(MotorB, Backward, 0);
    }else if(distances[1] < 100 && distances[0] < 100+20 ){
      motorobject.set_speed(MotorA, Backward, 150);
      motorobject.set_speed(MotorB, Forward, 150);
      delay(4725/2);
    }else if(distances[1] < 100 && distances[2] < 100+20 ){
      motorobject.set_speed(MotorA, Forward, 150);
      motorobject.set_speed(MotorB, Backward, 150);
      delay(4725/2);
    }else if(distances[1] < 100){
      motorobject.set_speed(MotorA, Backward, 150);
      motorobject.set_speed(MotorB, Forward, 150);
      delay(4725);
    }
    else{
      motorobject.set_speed(MotorA, Backward, 150);
      motorobject.set_speed(MotorB, Backward, 150);
    }
    delay(50);
  }
  vTaskDelete(NULL);
}
void sensingTask(void* nth){
  while (true)
  {
    if (sensor.isSampleDone())
      {
        sensor.readOutputRegs();

        amplitudes[sensor.channelUsed] = sensor.amplitude;
        distances[sensor.channelUsed] = sensor.distanceMillimeters;

        if (sensor.channelUsed == 2)
        {
            Serial.print("left:  " + String(distances[0]/10.0) + "cm, " );
            Serial.print("front: " + String(distances[1]/10.0) + "cm, " );
            Serial.println("right: " + String(distances[2]/10.0) + "cm " ); 
            //awsobject.publishMessage(distances);       
        }
        sensor.nextChannel();
        sensor.startSample();
    }
    delay(500);
  }
  vTaskDelete(NULL);

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
  // awsobject.connectAWS();
  // awsobject.stayConnected();
  xTaskCreate(sensingTask,"SensorTaskFunc",2*1024,NULL,2,NULL);
  delay(1000);
  xTaskCreate(motorTask,"MotorTaskFunc",2*1024,NULL,1,NULL);

}

void loop(){
  delay(500);
}

