#include <Arduino.h>

struct CONF_
{
    bool EN_MECANUM_WHEEL = true;
    const double WHEEL_SEPARATION = 0.2;                                              // m
    const double WHEEL_DISTANCE = 0.146;                                              // m
    const double WHEEL_DIAMETER = 0.096;                                              // m
    const double MOTOR_MAX_PPR = 3960;                                                // motor max speed in pulses per round
    const double MPS_TO_RPM_FACTOR = 60 / (M_PI * WHEEL_DIAMETER);                    //
    const double MOTOR_MAX_SPEED_IN_RPM = 111;                                        // rpm
    const double MOTOR_MAX_SPEED_IN_MPS = MOTOR_MAX_SPEED_IN_RPM / MPS_TO_RPM_FACTOR; //
    const double ANGULAR_VELOCITY_FACTOR = (WHEEL_DISTANCE * WHEEL_DISTANCE + WHEEL_SEPARATION * WHEEL_SEPARATION) / (2 * WHEEL_SEPARATION);
    const double MAX_ANGULAR_VELOCITY = (2 * MOTOR_MAX_SPEED_IN_RPM) / (MPS_TO_RPM_FACTOR * WHEEL_SEPARATION);
    const int32_t PWM_RESOLUTION_SET = 16;
    const uint32_t PWM_MAX_VAL = pow(2, PWM_RESOLUTION_SET) - 1;
    const uint32_t VEL_CAL_CYCLE = 10;    // in ms
    const uint32_t PID_SAMPLE_CYCLE = 10; // in ms
};

CONF_ CONFIG;
