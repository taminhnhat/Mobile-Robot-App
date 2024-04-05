#include <Arduino.h>

// system status
bool CRC_Enable = true;              //
bool Motor_Enable = false;           //
bool EN_MECANUM_WHEEL = false;       //
bool EN_VELOCITY_LOG = false;        //
bool EN_CURRENT_LOG = false;         //
bool EN_IMU_LOG = false;             //
bool IMU_AVAILABLE = false;          //
bool POWER_METTER_AVAILABLE = false; //
bool ledState = true;                //
// robot control constant
const int32_t PWM_RESOLUTION_SET = 16;                       //
const uint32_t PWM_MAX_VAL = pow(2, PWM_RESOLUTION_SET) - 1; //
const uint32_t LOG_CYCLE = 50;                               // in ms
const uint32_t CURRENT_CAL_CYCLE = 2;                        // in ms
const uint32_t MOTOR_CYCLE = 10;                             // in ms
const uint32_t BAT_VOL_CAL_CYCLE = 100;                      // in ms
const uint32_t DIS_VOL_CAL_CYCLE = 10;                       // in ms
const uint32_t MPU_CAL_CYCLE = 10000;                        // in us
const uint32_t MPU_SAMP_TIME = 10;                           // sampling time in ms
const uint32_t PID_SAMPLE_CYCLE = 20;                        // in ms
const uint32_t STATUS_CMD_CYCLE = 100;                       // in ms
uint32_t last_time_call_status = 0;                          // in ms
// robot hardware constant
const uint32_t DEFAULT_VEL_TIMEOUT = 1000;      //
const double WHEEL_SEPARATION = 0.205;          // m
const double WHEEL_DISTANCE = 0.146;            // m
const double WHEEL_DIAMETER = 0.096;            // m
const double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // m
const double MOTOR_PPR = 11 * 4 * 90;           // motor encoder pulses per round (2 wires, gear ratio 90:1)
// factor for fast computing
const double MPS_TO_RPM_FACTOR = 60 / (M_PI * WHEEL_DIAMETER);                                           // m/s to round/min factor
const double RPM_TO_MPS_FACTOR = (M_PI * WHEEL_DIAMETER) / 60;                                           //
const double MPS_TO_RAD_FACTOR = 2 / WHEEL_DIAMETER;                                                     //
const double VOL_TO_PWM_SIGNAL_OUTPUT_FACTOR = PWM_MAX_VAL / 12;                                         //
const double PULSE_TO_RAD_FACTOR = (2 * M_PI) / MOTOR_PPR;                                               //
const double MOTOR_MAX_SPEED_IN_RPM = 111;                                                               // rpm
const double MOTOR_MAX_SPEED_IN_MPS = MOTOR_MAX_SPEED_IN_RPM * RPM_TO_MPS_FACTOR;                        //
const double MOTOR_MAX_SPEED_IN_RAD = MOTOR_MAX_SPEED_IN_MPS * MPS_TO_RAD_FACTOR;                        //
const double MOTOR_ALLOW_MAX_SPEED_IN_MPS = 0.4;                                                         //
const double MOTOR_ALLOW_MAX_SPEED_IN_RAD = MOTOR_ALLOW_MAX_SPEED_IN_MPS * MPS_TO_RAD_FACTOR;            //
const double ANGULAR_VELOCITY_FACTOR = WHEEL_SEPARATION / 2;                                             //
const double PPMS_TO_MPS_FACTOR = 60000.0 * RPM_TO_MPS_FACTOR / MOTOR_PPR;                               // pulse/milisencond to m/s factor
const double ANALOG_INPUT_TO_VOLTAGE_FACTOR = 3.3 * 5 / 1023;                                            //
const double MAX_ANGULAR_VELOCITY = (2 * MOTOR_MAX_SPEED_IN_RPM * RPM_TO_MPS_FACTOR) / WHEEL_SEPARATION; //
