#include "wifi_bluetooth.h"
#include "globals.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <Arduino.h>
#include "pid.h"

// Section 2: WiFiBluetooth

// setup WiFi
void WiFiBluetooth::begin(wifi_mode_t mode, char *ssid, char *password)
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

WiFiBluetooth::WiFiBluetooth()
{
}

WiFiBluetooth &WiFiBluetooth::getInstance()
{
    static WiFiBluetooth instance;
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

/* COMMANDS:

    J1 x y -> Yaw and Throttle
    J2 x y -> Roll and Pitch
    arm -> Arm
    calibrate -> Calibrate
    stop -> Stop
    packed Kp x Kd y Ki z -> PID gains (grouped together)
    Kp x -> Kp
    Ki x -> Ki
    Kd x -> Kd
    pitch x -> Pitch invert or not
    roll x -> Roll invert or not
    yaw x -> Yaw invert or not
    save -> Save to eeprom

*/

void WiFiBluetooth::processIncoming(String data, volatile control_t &prevControls, volatile state_t &state)
{
    try
    {
        int len = data.length();
        control_t rControls = copyControl(prevControls);
        float recv_kp = 0.0f, recv_ki = 0.0f, recv_kd = 0.0f;
        data.trim();
#if !defined(USE_RC)
        if (data.startsWith("J1"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex, data.indexOf(' ', sIndex + 1)).toDouble();
            sIndex = data.indexOf(' ', sIndex + 1);
            int y = data.substring(sIndex).toDouble();

            rControls.yaw = x * 30.0;
#if defined(BRUSHED)
            rControls.throttle += y * 5000;
#else
            rControls.throttle += y * 100;
#endif
        }
        else if (data.startsWith("J2"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex, data.indexOf(' ', sIndex + 1)).toDouble();
            sIndex = data.indexOf(' ', sIndex + 1);
            int y = data.substring(sIndex).toDouble();

            rControls.roll = x * 30.0;
            rControls.pitch = y * 30.0;
        }
#endif
        if (data.startsWith("arm"))
        {
            state.arm = true;
        }
        else if (data.startsWith("calibrate"))
        {
            state.calibrate = true;
        }
        else if (data.startsWith("stop"))
        {
            rControls = (control_t)IDLE;
            ;
        }
        else if (data.startsWith("Kp"))
        {
            int sIndex = data.indexOf(' ');
            recv_kp = data.substring(sIndex).toDouble();
            state._Kp = recv_kp;
        }
        else if (data.startsWith("Ki"))
        {
            int sIndex = data.indexOf(' ');
            recv_ki = data.substring(sIndex).toDouble();
            state._Ki = recv_ki;
        }
        else if (data.startsWith("Kd"))
        {
            int sIndex = data.indexOf(' ');
            recv_kd = data.substring(sIndex).toDouble();
            state._Kd = recv_kd;
        }
        else if (data.startsWith("pitch"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex).toDouble();
            state.inverted_pitch = x;
        }
        else if (data.startsWith("roll"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex).toDouble();
            state.inverted_roll = x;
        }
        else if (data.startsWith("yaw"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex).toDouble();
            state.inverted_yaw = x;
        }
        else if (data.startsWith("packed"))
        {
            int sIndex = data.indexOf(' ');
            recv_kp = data.substring(sIndex, data.indexOf(' ', sIndex + 1)).toDouble();
            sIndex = data.indexOf(' ', sIndex + 1);
            recv_kd = data.substring(sIndex, data.indexOf(' ', sIndex + 1)).toDouble();
            sIndex = data.indexOf(' ', sIndex + 1);
            recv_ki = data.substring(sIndex).toDouble();
            state._Kp = recv_kp;
            state._Kd = recv_kd;
            state._Ki = recv_ki;
        }
    }
    catch (...)
    {
        // TODO
    }
}
void WiFiBluetooth::receive(volatile control_t &prevControls, volatile state_t &state)
{
    // TODO
    String data;
    if (client.connected())
    {
        Globals::getInstance().setError(0, 2);
        // TODO
        if (client.available())
        {
            data = client.readStringUntil('\r');
        }
    }
    else
    {
        client = server.available();
        if (!client)
        { // disconnected
            Globals::getInstance().setError(3, 2);
#if !defined(USE_RC)
            prevControls.throttle = 0; // handle failsafe here (midflight disconnect)
#endif
        }
    }
    if (data.length() > 0)
        processIncoming(data, prevControls, state);
}