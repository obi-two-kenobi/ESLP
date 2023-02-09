/**
 * ESP32 AWS Library
 * 
 * Functions to get the crawler coordinates from the Camera over AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "AWS.h"
#include <iostream>
#include <mutex>

/* The MQTT topics that this device should publish/subscribe to */
#define AWS_IOT_PUBLISH_TOPIC   "greengrass/group05"  //this is the topic that was used in milestone 1 to test the connection between the rover and the cloud
#define AWS_IOT_SUBSCRIBE_TOPIC1 "esp32/rover"    // this topic is used to get the rover coordinates from the camera through the cloud
#define AWS_IOT_SUBSCRIBE_TOPIC2 "esp32/target"  // this topic is used to get the target coordinates from the camera through the cloud

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

extern int correctionAngle; //this variable is used to store the angle between the rover and the target
extern int LinearDistance;  //this variable is used to store the distance between the rover and the target

myawsclass::myawsclass() {

}

extern coordinate mytarget;            
extern coordinatesWithAngle roverLoc;   

void messageHandler(String &topic, String &payload) {
  // argument 1: topic, it is a String reference to the topic string
  // argument 2: payload, it is a String reference to the payload string which is the message body of the MQTT message
  // messageHandler is called when a message is received on a topic that the client is subscribed to

  using namespace std;

  //get x any y from the target topic
  if (topic == "esp32/target") {
    #ifdef DEBUG
    Serial.println("incoming: " + topic + " - " + payload);
    #endif
    auto xvalue = payload.substring(payload.indexOf("\"(")+2, payload.indexOf(",")); //get the x value
    auto yvalue = payload.substring(payload.indexOf(",") + 1, payload.indexOf(")")); //get the y value
    // save the target coordinates to the mytarget struct
    mytarget.x = xvalue.toInt();
    mytarget.y = yvalue.toInt();
  }

  //get x any y from the rover topic
  if (topic == "esp32/rover") {
    #ifdef DEBUG
    Serial.println("incoming: " + topic + " - " + payload);
    #endif
    auto xvalue = payload.substring(payload.indexOf("[(") + 2, payload.indexOf(",")); //get the x value
    auto yvalue = payload.substring(payload.indexOf(",") + 1, payload.indexOf("),")); //get the y value
    auto degvalue = payload.substring(payload.lastIndexOf(",") + 1, payload.indexOf("]}")); //get the deg value
    // save the rover coordinates to the roverLoc struct
    roverLoc.x = xvalue.toInt();
    roverLoc.y = yvalue.toInt();
    roverLoc.deg = degvalue.toInt();
  }

  //get angle between rover and target using the equation: angle = arctan(y2-y1/x2-x1), then confine the angle to the range of -180 to 180 
  auto angleRoverCorrection = roverLoc.deg > 180 ? roverLoc.deg - 360  : roverLoc.deg;
  auto diffAngle = (180.0/3.141592)* atan2(mytarget.y - roverLoc.y, mytarget.x - roverLoc.x)-180; 
  correctionAngle = 180-diffAngle-angleRoverCorrection;   
  correctionAngle = (correctionAngle <=180 ? correctionAngle : correctionAngle-360);

  //get distance between rover and target using the equation: distance = sqrt((x2-x1)^2 + (y2-y1)^2)
  LinearDistance = sqrt(pow(mytarget.x - roverLoc.x, 2) + pow(mytarget.y - roverLoc.y, 2));
}

void myawsclass::stayConnected() {
  client.loop();
}

void myawsclass::connectAWS() {
  /* Configure the WiFi connection */
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  #ifdef DEBUG
  Serial.print("Connecting to Wi-Fi");
  #endif

  // Wait for connection 
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    #ifdef DEBUG
    Serial.print(".");
    #endif
  }

  #ifdef DEBUG
  Serial.print("CONNECTED...!\n");
  #endif

  /* Configure WiFiClientSecure to use the AWS IoT device credentials */
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  /* Connect to the MQTT broker on the AWS endpoint we defined earlier */
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  /* Create a message handler */
  client.onMessage(messageHandler);

  #ifdef DEBUG
  Serial.print("Connecting to AWS IOT");
  #endif

  /* Connect to AWS IoT */
  while (!client.connect(THINGNAME)) {
    #ifdef DEBUG
    Serial.print(".");
    #endif
    delay(100);
  }

  // Check if the connection was successful
  if(!client.connected())
  {
    #ifdef DEBUG
    Serial.println("AWS IoT Timeout!");
    #endif
    return;
  }

  /* Subscribe to a topic */
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC1);
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC2);

  #ifdef DEBUG
  Serial.println("AWS IoT Connected!");
  #endif
}

void myawsclass::publishMessage(int16_t sensorValue[3]) {
  // publish the sensor values to the topic
  // sensorValue is an array of 3 values (x, y, z)

  // create a JSON document
  StaticJsonDocument<200> doc;
  // read the sensor values and save them to the doc
  doc["x"] = sensorValue[0];
  doc["y"] = sensorValue[1];
  doc["z"] = sensorValue[2];
  // serialize the doc to a jsonBuffer
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  // publish the jsonBuffer to the topic
  client.publish("esp32/correction", jsonBuffer);
}

void myawsclass::publishMessage(String msg) {
  // publish a message to the topic
  // msg is a string

  StaticJsonDocument<200> doc;
  doc["msg"] = msg;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); /* print to client */

  // publish the jsonBuffer to the topic
  client.publish("esp32/correction", jsonBuffer);
}

myawsclass awsobject = myawsclass();  /* creating an object of class aws */