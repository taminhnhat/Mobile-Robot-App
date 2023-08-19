#include <Arduino.h>

struct CONF_
{
    bool CRC_Enable = true;
    bool EN_MECANUM_WHEEL = false;
    bool EN_VELOCITY_LOG = false;                                                     //
    const uint32_t DEFAULT_VEL_TIMEOUT = 1000;                                        //
    const double WHEEL_SEPARATION = 0.205;                                            // m
    const double WHEEL_DISTANCE = 0.146;                                              // m
    const double WHEEL_DIAMETER = 0.096;                                              // m
    const double MOTOR_PPR = 3960;                                                    // motor encoder pulses per round
    const double MPS_TO_RPM_FACTOR = 60 / (M_PI * WHEEL_DIAMETER);                    //
    const double RPM_TO_MPS_FACTOR = (M_PI * WHEEL_DIAMETER) / 60;                    //
    const double PULSE_TO_RAD_FACTOR = (2 * M_PI) / MOTOR_PPR;                        //
    const double MOTOR_MAX_SPEED_IN_RPM = 111;                                        // rpm
    const double MOTOR_MAX_SPEED_IN_MPS = MOTOR_MAX_SPEED_IN_RPM * RPM_TO_MPS_FACTOR; //
    const double MOTOR_ALLOW_MAX_SPEED_IN_MPS = 0.4;                                  //
    const double ANGULAR_VELOCITY_FACTOR = WHEEL_SEPARATION / 2;
    const double MAX_ANGULAR_VELOCITY = (2 * MOTOR_MAX_SPEED_IN_RPM * RPM_TO_MPS_FACTOR) / WHEEL_SEPARATION;
    const int32_t PWM_RESOLUTION_SET = 16;
    const uint32_t PWM_MAX_VAL = pow(2, PWM_RESOLUTION_SET) - 1;
    const uint32_t VEL_CAL_CYCLE = 10;      // in ms
    const uint32_t BAT_VOL_CAL_CYCLE = 100; // in ms
    const uint32_t DIS_VOL_CAL_CYCLE = 10;  // in ms
    const uint32_t PID_SAMPLE_CYCLE = 10;   // in ms
    const uint32_t STATUS_CMD_CYCLE = 100;  // in ms
    uint32_t last_time_call_status = 0;     // in ms
};

CONF_ CONFIG;
