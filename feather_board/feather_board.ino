#include <Arduino.h>

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <WebSocketsServer.h>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_MotorShield.h>

#include "debug.h"
#include "server.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorVert1 = AFMS.getMotor(1);
Adafruit_DCMotor *motorVert2 = AFMS.getMotor(2);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);

//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = "NETGEAR33";
char* sta_password = "freestreet163";

void setupPins() {
    // setup Serial and actuators
    Serial.begin(115200);
    DEBUG("Started serial.");
    AFMS.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    DEBUG("Setup pins");
}

void setmotors(const uint8_t payload[10]) {
    digitalWrite(LED_BUILTIN, payload[9]);
    motorVert1->run(payload[1]);
    motorVert1->setSpeed(payload[2]);
    motorVert2->run(payload[3]);
    motorVert2->setSpeed(payload[4]);
    motorLeft->run(payload[5]);
    motorLeft->setSpeed(payload[6]);
    motorRight->run(payload[7]);
    motorRight->setSpeed(payload[8]);
}

void stop() {
    const static uint8_t off[10] = {0,0,0,0,0,0,0,0,0,0};
    setmotors(off);
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {
    char ipstr[16];

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;

        case WStype_CONNECTED: 
            // IPAddress ip = webSocket.remoteIP(id);
            // sprintf(ipstr, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            DEBUG("Web socket connected, id = ", id);
            // DEBUG("  from: ", ipstr);
            DEBUG("  url: ", (char *)payload);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;

        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);
            if (payload[0] == '~' && length == 10) 
              setmotors(payload);
            break;

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);
            break;
    }
//    VL53L0X_RangingMeasurementData_t measure;
//    lox.rangingTest(&measure, false);
//    DEBUG("Lidar Value : ", measure.RangeMilliMeter);
//    wsSendInt(id, measure.RangeMilliMeter);    
}

void setup() {
    setupPins();
    
    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
    }

//    if (!lox.begin()) {
//      Serial.println(F("Failed to boot VL53L0X"));
//      while(1);
//    }

    setupSTA(sta_ssid, sta_password);
    //setupAP(ap_ssid, ap_password);
    setupWS(webSocketEvent);
    stop();
}

void loop() {
    wsLoop();
}
