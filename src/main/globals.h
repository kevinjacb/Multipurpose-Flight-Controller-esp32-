#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <mutex>
#include <EEPROM.h>

// LED Pins and Error Codes
#define RED_L 25
#define GREEN_L 18
#define BLUE_L 26
#define IMU_ERROR 1

#define ENABLE_DEBUG
// wifi control enable
#define USE_WIFI
// disables radio control over the drone via wifi
#define USE_RC 35 // pin number
#define NUM_CHANNELS 8

// ####### SENSORS #######
// IMU Address
#define SEL_MPU6050 // selects mpu6050 sensor
// #define SEL_MPU9250
#define ADDR 0x68
#define BATT_CELLS 3   // number of cells in battery (3S,4S, etc.)
#define BATT 4         // battery pin
#define BATT_DIV 7.033 // voltage divider
#define ADC_RES 4095   // adc resolution

// pressure sensor(for altitude)
// #define BARO // enter pin here
// ultrasonic senosr(lower altitude maintainence)
// #define HCSR04
#define TRIG_PIN 2
#define ECHO_PIN 15

// PID Gains (default)
#define pre_Kp 3.0
#define pre_Ki 0.05
#define pre_Kd 1.0
#define pre_dt 0.004 // s

// WiFi AP Credentials & settings
#define AP_SSID "DRONEY"
#define AP_PSWD "123456789"
#define TIMEOUT 60000 // wifi timeout in ms (1 min)

// drone types TODO
#define BRUSHLESS // uncomment for brushless

#define QUAD_X
// #define HEX

// motor pins (currently only for quad x and hex configs)
#define MOTOR1 13
#define MOTOR2 12
#define MOTOR3 14
#define MOTOR4 27
#define MOTOR5 33
#define MOTOR6 32

// PWM frequencies
#define PWM_FREQ_BRUSHLESS 50 // Hz
#define PWM_FREQ_BRUSHED 1200 // Hz
#define PWM_RES_BRUSHED 16    // bits
#define PWM_RES_BRUSHLESS 10  // bits

// IDLE control
#if defined(BRUSHLESS)
#define IDLE_VALUE 1000
#define IDLE                               \
    {                                      \
        1000, 1000, 1000, 1000, 1000, 1000 \
    }
#else
#define IDLE_VALUE 0
#define IDLE       \
    {              \
        0, 0, 0, 0 \
    }
#endif
// EEPROM settings
#define EEPROM_SIZE 65
#define KP_ADDR 0
#define KI_ADDR 4
#define KD_ADDR 8
#define YAW_KP_ADDR 12
#define YAW_KI_ADDR 16
#define YAW_KD_ADDR 20
#define ACC_X_BIAS_ADDR 24
#define ACC_Y_BIAS_ADDR 28
#define ACC_Z_BIAS_ADDR 32
#define GYRO_X_BIAS_ADDR 36
#define GYRO_Y_BIAS_ADDR 40
#define GYRO_Z_BIAS_ADDR 44
#define MAG_X_BIAS_ADDR 48
#define MAG_Y_BIAS_ADDR 52
#define MAG_Z_BIAS_ADDR 56
#define PITCH_INVERT_ADDR 60
#define ROLL_INVERT_ADDR 61
#define YAW_INVERT_ADDR 62

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 5

// data transfer settings
#define SEPARATOR " "

// control data struct
typedef struct control
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float aux1;
    float aux2;
} control_t;

control_t copyControl(volatile control_t &control);

// motor outputs struct
typedef struct outputs
{
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
    uint16_t motor5;
    uint16_t motor6;
} output_t;

typedef struct state
{
    bool inverted_pitch, inverted_roll, inverted_yaw;
    bool arm, stop;
    bool calibrate;
    float _Kp, _Ki, _Kd;
    float _Yaw_Kp, _Yaw_Ki, _Yaw_Kd;
} state_t;

class Globals
{
private:
    uint8_t error;
    uint8_t handler;
    std::mutex errorMutex;
    Globals();

public:
    static constexpr uint8_t LED_PINS[3] = {RED_L, GREEN_L, BLUE_L};
    static Globals &getInstance();
    uint8_t getError();
    void setError(uint8_t error, uint8_t section);
};

class EEPROMHandler
{
    /*

    ! Handles float only for now !

    bytes:
    0-3: Kp
    4-7: Ki
    8-11: Kd
    12-15: YawKp
    16-19: YawKi
    20-23: YawKd
    24-27: AccXBias
    28-31: AccYBias
    32-35: AccZBias
    36-39: GyroXBias
    40-43: GyroYBias
    44-47: GyroZBias
    48-51: MagXBias
    52-55: MagYBias
    56-59: MagZBias
    60: inverted_pitch
    61: inverted_roll
    62: inverted_yaw

    */
private:
    EEPROMHandler();

public:
    static EEPROMHandler &getInstance();
    void write(uint16_t address, float data);
    float read(uint16_t address);
    void begin();
};

#endif // GLOBALS_H