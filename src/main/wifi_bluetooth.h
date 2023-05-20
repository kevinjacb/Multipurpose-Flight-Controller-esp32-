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

public:
    static WiFiBluetooth &getInstance();
    control_t processIncoming(String data, control_t prevControls, state_t &state);
    void connect(char *ssid, char *password); // connect to an external network (STA mode)
    void disconnect();                        // disconnect from an external network/ devices
    void send(const char *data);
    void begin(wifi_mode_t mode = WIFI_AP, char *ssid = AP_SSID, char *password = AP_PSWD);
    control_t receive(control_t prevControls, state_t &state);
};

#endif // WB_H