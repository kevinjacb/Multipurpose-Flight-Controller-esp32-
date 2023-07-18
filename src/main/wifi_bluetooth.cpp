#include "wifi_bluetooth.h"
#include "globals.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <Arduino.h>
#include "pid.h"

#define print(x) Serial.print(x)
#define println(x) Serial.println(x)
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
    if (!client.connected())
        return;
    client.printf(data);
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
        float recv_kp = 0.0f, recv_ki = 0.0f, recv_kd = 0.0f;
        data.trim();
#if !defined(USE_RC)
        if (data.startsWith("J1"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex, data.indexOf(' ', sIndex + 1)).toFloat();
            sIndex = data.indexOf(' ', sIndex + 1);
            int y = data.substring(sIndex).toFloat();

            prevControls.yaw = x * 30.0;
#if MODE == MICRO_QUAD_X
            prevControls.throttle += y * 5000;
#else
            prevControls.throttle += y * 100;
#endif
        }
        else if (data.startsWith("J2"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex, data.indexOf(' ', sIndex + 1)).toFloat();
            sIndex = data.indexOf(' ', sIndex + 1);
            int y = data.substring(sIndex).toFloat();

            prevControls.roll = x * 30.0;
            prevControls.pitch = y * 30.0;
        }
#endif
        bool yaw = false;
        if (data.startsWith("Yaw") && !data.startsWith("Yaw packed"))
        {
            data = data.substring(data.indexOf(' ') + 1);
            yaw = true;
        }
        else if (data.startsWith("Pitch/Roll") && !data.startsWith("Pitch/Roll packed"))
        {
            data = data.substring(data.indexOf(' ') + 1);
        }

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
            state.stop = true;
        }
        else if (data.startsWith("Kp"))
        {
            int sIndex = data.indexOf(' ');
            recv_kp = data.substring(sIndex).toFloat();
            if (!yaw)
                state._Kp = recv_kp;
            else
                state._Yaw_Kp = recv_kp;
        }
        else if (data.startsWith("Ki"))
        {
            int sIndex = data.indexOf(' ');
            recv_ki = data.substring(sIndex).toFloat();
            if (!yaw)
                state._Ki = recv_ki;
            else
                state._Yaw_Ki = recv_ki;
        }
        else if (data.startsWith("Kd"))
        {
            int sIndex = data.indexOf(' ');
            recv_kd = data.substring(sIndex).toFloat();
            if (!yaw)
                state._Kd = recv_kd;
            else
                state._Yaw_Kd = recv_kd;
        }
        else if (data.startsWith("pitch"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex).toFloat();
            state.inverted_pitch = x;
        }
        else if (data.startsWith("roll"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex).toFloat();
            state.inverted_roll = x;
        }
        else if (data.startsWith("yaw"))
        {
            int sIndex = data.indexOf(' ');
            int x = data.substring(sIndex).toFloat();
            state.inverted_yaw = x;
        }
        else if (data.startsWith("Pitch/Roll packed"))
        {
            // format of data = Pitch/Roll packed Kp value Kd value Ki value
            // Serial.println(data); // debug
            // reset the string to the start of the packed data
            int sIndex = data.indexOf(' ', data.indexOf(' ', data.indexOf(' ') + 1) + 1); // start of the Kp value
            recv_kp = data.substring(sIndex + 1, data.indexOf(' ', sIndex + 1)).toFloat();
            sIndex = data.indexOf(' ', data.indexOf(' ', sIndex + 1) + 1);
            recv_ki = data.substring(sIndex + 1, data.indexOf(' ', sIndex + 1)).toFloat();
            sIndex = data.indexOf(' ', data.indexOf(' ', sIndex + 1) + 1);
            recv_kd = data.substring(sIndex + 1).toFloat();
            state._Kp = recv_kp;
            state._Kd = recv_kd;
            state._Ki = recv_ki;
        }
        else if (data.startsWith("Yaw packed"))
        {
            // format of data = Yaw packed Kp value Kd value Ki value
            int sIndex = data.indexOf(' ', data.indexOf(' ', data.indexOf(' ') + 1) + 1); // start of the Kp value
            recv_kp = data.substring(sIndex + 1, data.indexOf(' ', sIndex + 1)).toFloat();
            sIndex = data.indexOf(' ', data.indexOf(' ', sIndex + 1) + 1);
            recv_ki = data.substring(sIndex + 1, data.indexOf(' ', sIndex + 1)).toFloat();
            sIndex = data.indexOf(' ', data.indexOf(' ', sIndex + 1) + 1);
            recv_kd = data.substring(sIndex + 1).toFloat();
            state._Yaw_Kp = recv_kp;
            state._Yaw_Kd = recv_kd;
            state._Yaw_Ki = recv_ki;
        }
        else if (data.startsWith("save"))
        { // save settings to eeprom
            // write Kp, Ki, Kd, yaw_kp, yaw_ki , yaw_kd inverted_pitch, inverted_roll, inverted_yaw
            Globals::getInstance().setError(5, 2);
            lastWriteToEEPROM = millis();
            EEPROMHandler::getInstance().write(KP_ADDR, state._Kp);
            EEPROMHandler::getInstance().write(KI_ADDR, state._Ki);
            EEPROMHandler::getInstance().write(KD_ADDR, state._Kd);
            EEPROMHandler::getInstance().write(YAW_KP_ADDR, state._Yaw_Kp);
            EEPROMHandler::getInstance().write(YAW_KI_ADDR, state._Yaw_Ki);
            EEPROMHandler::getInstance().write(YAW_KD_ADDR, state._Yaw_Kd);
            EEPROMHandler::getInstance().write(PITCH_INVERT_ADDR, state.inverted_pitch);
            EEPROMHandler::getInstance().write(ROLL_INVERT_ADDR, state.inverted_roll);
            EEPROMHandler::getInstance().write(YAW_INVERT_ADDR, state.inverted_yaw);
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
    // reset the indicator for indicating eeprom write
    if (lastWriteToEEPROM != -1 && millis() - lastWriteToEEPROM > 2000)
    {
        Globals::getInstance().setError(0, 2);
        lastWriteToEEPROM = -1;
    }
    String data;
    if (client.connected())
    {
        if (Globals::getInstance().getError() != 5)
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