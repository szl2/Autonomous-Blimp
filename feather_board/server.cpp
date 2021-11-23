#include <Arduino.h>
#include "server.h"
#include "debug.h"

#include <WebServer.h>
#include <WiFi.h>
//#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include <WebSocketsServer.h>

#define STA_MAXTRIES 30

WebSocketsServer wsServer = WebSocketsServer(81);

//
// Setup //
//

void setupSTA(char* ssid, char* password) {
    DEBUG("Connecting to STA");
    WiFi.begin(ssid, password);

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED) {
        if (tries++ > STA_MAXTRIES) {
            DEBUG("  giving up.");
            return;
        }
        delay(500);
        DEBUG("  ... waiting");
    }

    IPAddress myIP = WiFi.localIP();
    DEBUG("STA IP address: ");
    DEBUG(myIP.toString());
}

void setupAP(char* ssid, char* password) {
    WiFi.softAP(ssid, password);

    IPAddress myIP = WiFi.softAPIP();
    DEBUG("AP IP address: ");
    DEBUG(myIP.toString());
//    DEBUG("Setting soft-AP configuration ... ");
//    DEBUG(WiFi.softAPConfig(myIP, gateway, subnet) ? "Ready" : "Failed!");

    DEBUG("Setting soft-AP ... ");
    DEBUG(WiFi.softAP("ESPsoftAP_01") ? "Ready" : "Failed!");

    DEBUG("Soft-AP IP address = ");
    DEBUG(WiFi.softAPIP());
}

void setupWS(ws_callback_t callback) {
    // start webSocket server
    wsServer.begin();
    wsServer.onEvent(callback);
}

void wsLoop() {
	wsServer.loop();
}

void wsSend(int id, char* txt) {
    wsServer.sendTXT(id, txt);
}

void wsSendInt(int id, uint16_t val) {
    uint8_t data[3] = {101, 0, 0};
    data[1] = val & 0xff;
    data[2] = ( val >> 8 ) & 0xff;

    wsServer.sendBIN(id, data, 3);
}
