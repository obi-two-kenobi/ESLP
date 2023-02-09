/**
 * @file
 * 
 * ESP32 AWS Library
 * 
 * Functions to get the crawler coordinates from the Camera over AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

#ifndef aws_h
#define aws_h

#include <MQTTClient.h>

struct coordinate{
  int x;
  int y;
};

struct coordinatesWithAngle: coordinate{
  int deg;
};

void messageHandler(String, String);
class myawsclass {
  public:
    myawsclass();

    void connectAWS();                            /* Initialize and connect to AWS */
    void publishMessage(int16_t sensorValue[3]);     /* Publish the values of the sensors */
    void publishMessage(String msg);
    void stayConnected();                         /* Maintain the connection */
};

extern myawsclass awsobject;

#endif