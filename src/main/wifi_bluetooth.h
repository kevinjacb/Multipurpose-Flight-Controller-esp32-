#ifndef WB_H
#define WB_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "globals.h"

class WiFiBluetooth
{
private:
    WiFiBluetooth(wifi_mode_t mode, char *ssid, char *password);
    WiFiBluetooth(const WiFiBluetooth &) = delete;
    static WiFiBluetooth instance;
    WiFiServer server;
    WiFiClient client;

public:
    static WiFiBluetooth &getInstance(wifi_mode_t mode = WIFI_AP, char *ssid = AP_SSID, char *password = AP_PSWD);
    void connect(char *ssid, char *password); // connect to an external network (STA mode)
    void disconnect();                        // disconnect from an external network/ devices
    void send(const char *data);
    control_t receive(control_t prevControls);
};

#endif // WB_H