#include "wifi_bluetooth.h"
#include "globals.h"
#include <WiFi.h>
#include <WiFiClient.h>

// Section 2: WiFiBluetooth

// setup WiFi
WiFiBluetooth::WiFiBluetooth(wifi_mode_t mode, char *ssid, char *password)
{
    WiFi.mode(mode);
    if (mode == WIFI_AP)
    { // access point mode
        WiFi.softAP(ssid, password);
    }
    else if (mode == WIFI_STA)
    {
        // station mode
        WiFi.begin(ssid, password);
        // else if (mode == WIFI_AP_STA) // access point and station mode
        //     WiFi.softAP(ssid, password);

        long lastTime = millis();
        Globals::getInstance().setError(3, 2);
        if (mode == WIFI_STA || mode == WIFI_AP_STA)
        {
            while (WiFi.status() != WL_CONNECTED)
            {
                if (millis() - lastTime > TIMEOUT)
                {
                    Globals::getInstance().setError(4, 2);
                    ESP.restart();
                    break;
                }
            }
        }
    }
    if (Globals::getInstance().getError() != 4)
        Globals::getInstance().setError(0, 2);
    server.begin(80);
    // }
}

WiFiBluetooth &WiFiBluetooth::getInstance(wifi_mode_t mode, char *ssid, char *password)
{
    static WiFiBluetooth instance(mode, ssid, password);
    return instance;
}

// for station mode
void WiFiBluetooth::connect(char *ssid, char *password)
{
    WiFi.disconnect();
    WiFi.begin(ssid, password);

    Globals::getInstance().setError(3, 2);
    long lastTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - lastTime > TIMEOUT)
        {
            Globals::getInstance().setError(4, 2);
            break;
        }
    }

    if (Globals::getInstance().getError() != 4)
        Globals::getInstance().setError(0, 2);
}

void WiFiBluetooth::disconnect()
{
    WiFi.disconnect();
}

void WiFiBluetooth::send(const char *data)
{
    // TODO
}

control_t WiFiBluetooth::receive(control_t prevControls)
{
    // TODO
    char *data;
    if (client.connected())
    {
        // TODO
        if (client.available())
        {
            client.readStringUntil('\r').toCharArray(data, 100);
        }
    }
    else
    {
        client = server.available();
        if (!client)
        { // disconnected
            Globals::getInstance().setError(4, 2);
            return (control_t)IDLE; // handle failsafe here (midflight disconnect)
        }
    }

    if (data && strlen(data) > 0)
    {
        int len = strlen(data);
        control_t rControls;
        try // extracts received data
        {
            char *token = strtok(data, SEPARATOR);
            rControls.throttle = atof(token);
            token = strtok(NULL, SEPARATOR);
            rControls.roll = atof(token);
            token = strtok(NULL, SEPARATOR);
            rControls.pitch = atof(token);
            token = strtok(NULL, SEPARATOR);
            rControls.yaw = atof(token);
            token = strtok(NULL, SEPARATOR); // rest of the data // todo
            return rControls;
        }
        catch (...)
        {
            return prevControls;
        }
    }
    return prevControls;
}