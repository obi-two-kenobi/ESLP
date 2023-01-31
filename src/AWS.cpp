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
#define AWS_IOT_PUBLISH_TOPIC   "greengrass/group05" 
#define AWS_IOT_SUBSCRIBE_TOPIC1 "esp32/rover"
#define AWS_IOT_SUBSCRIBE_TOPIC2 "esp32/target"

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

extern int correctionAngle;
extern int LinearDistance;

myawsclass::myawsclass() {

}

extern target mytarger;
extern rover roverLoc;

void messageHandler(String &topic, String &payload) {
  using namespace std;

  //get x any y from the target topic
  if (topic == "esp32/target") {
    Serial.println("incoming: " + topic + " - " + payload);
    auto xvalue = payload.substring(payload.indexOf("\"(")+2, payload.indexOf(",")); //get the x value
    auto yvalue = payload.substring(payload.indexOf(",") + 1, payload.indexOf(")")); //get the y value
    mytarger.x = xvalue.toInt();
    mytarger.y = yvalue.toInt();
    

  }
  if (topic == "esp32/rover") {
    Serial.println("incoming: " + topic + " - " + payload);
    auto xvalue = payload.substring(payload.indexOf("[(") + 2, payload.indexOf(",")); //get the x value
    auto yvalue = payload.substring(payload.indexOf(",") + 1, payload.indexOf("),")); //get the y value
    auto degvalue = payload.substring(payload.lastIndexOf(",") + 1, payload.indexOf("]}")); //get the deg value
    roverLoc.x = xvalue.toInt();
    roverLoc.y = yvalue.toInt();
    roverLoc.deg = degvalue.toInt();
    

  }
  //std::cout << "rover (X,Y,theta): " << roverLoc.x << ", " << roverLoc.y << ", " << roverLoc.deg << "|---> target (X,Y): " << mytarger.x << ", " << mytarger.y << std::endl;
  //get angle between rover and target
  auto angleRoverCorrection = roverLoc.deg > 180 ? roverLoc.deg - 360  : roverLoc.deg;
  auto diffAngle = (180.0/3.141592)* atan2(mytarger.y - roverLoc.y, mytarger.x - roverLoc.x)-180; 
  correctionAngle = 180-diffAngle-angleRoverCorrection;
  
  //get angle between rover and target
  correctionAngle = (correctionAngle <=180 ? correctionAngle : correctionAngle-360) ;

  //get distance between rover and target
  LinearDistance = sqrt(pow(mytarger.x - roverLoc.x, 2) + pow(mytarger.y - roverLoc.y, 2));

  //publish for debugging; remove it later.
  // int16_t p[3] = {correctionAngle, LinearDistance, correctionAngle};
  // awsobject.publishMessage(p);
}

void myawsclass::stayConnected() {
  client.loop();
}


void myawsclass::connectAWS() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }

  Serial.print("CONNECTED...!\n");

  /* Configure WiFiClientSecure to use the AWS IoT device credentials */
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  /* Connect to the MQTT broker on the AWS endpoint we defined earlier */
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  /* Create a message handler */
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  /* Subscribe to a topic */
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC1);
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC2);

  Serial.println("AWS IoT Connected!");
}

void myawsclass::publishMessage(int16_t sensorValue[3]) {

  StaticJsonDocument<200> doc;
  //doc["time"] = millis();
  doc["x"] = sensorValue[0];
  doc["y"] = sensorValue[1];
  doc["z"] = sensorValue[2];
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); /* print to client */

  client.publish("esp32/correction", jsonBuffer);
}
void myawsclass::publishMessage(String msg) {
  StaticJsonDocument<200> doc;
  //doc["time"] = millis();
  doc["msg"] = msg;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); /* print to client */

  client.publish("esp32/correction", jsonBuffer);
}

myawsclass awsobject = myawsclass();  /* creating an object of class aws */


