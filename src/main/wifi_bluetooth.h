#ifndef WB_H
#define WB_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "globals.h"

class WiFiBluetooth
{
private:
    WiFiBluetooth();
    WiFiBluetooth(const WiFiBluetooth &) = delete;
    WiFiServer server;
    WiFiClient client;
    long lastWriteToEEPROM = -1;

public:
    static WiFiBluetooth &getInstance();
    void processIncoming(String data, volatile control_t &prevControls, volatile state_t &state);
    void connect(char *ssid, char *password); // connect to an external network (STA mode)
    void disconnect();                        // disconnect from an external network/ devices
    void send(const char *data);
    void begin(wifi_mode_t mode = WIFI_AP, char *ssid = AP_SSID, char *password = AP_PSWD);
    void receive(volatile control_t &prevControls, volatile state_t &state);
};

#endif // WB_H