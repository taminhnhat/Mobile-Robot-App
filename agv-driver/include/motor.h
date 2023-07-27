#include <Arduino.h>
#include "configuration.h"

// define control mode
#define IF_POSITION_CONTROL_MODE 1
#define IF_VELOCITY_CONTROL_MODE 2
#define IF_LOCKED_MODE 0
#define IF_FLOATING_MODE 3

// define motor 1
#define MOTOR_1_A PA7
#define MOTOR_1_B PA6
#define MOTOR_1_PWM PB10
#define MOTOR_1_DIR PB1
// define motor 2
#define MOTOR_2_A PA5
#define MOTOR_2_B PA4
#define MOTOR_2_PWM PB4
#define MOTOR_2_DIR PB5
// define motor 3
#define MOTOR_3_B PA15
#define MOTOR_3_A PB3
#define MOTOR_3_PWM PB15
#define MOTOR_3_DIR PA8
// define motor 4
#define MOTOR_4_B PA11
#define MOTOR_4_A PA12
#define MOTOR_4_PWM PB13
#define MOTOR_4_DIR PB14

// define sensor
#define DIS_SEN PA1
#define FRONT_LIGHT PC13
#define BATTERY_SENSOR PB0

// struct CONF_ CONFIG;

class MotorControl
{
private:
#define DIRECT 0
#define REVERSE 1
    // Motor parameters
    String id;
    int64_t a;              // a channel count
    int64_t b;              // b channel count
    int64_t p_ins;          // instant encoder pulse value
    int64_t p_pre;          // previous encoder pulse value
    int64_t p_set;          // setpoint encoder pulse value
    int64_t d_p = 0;        //
    int64_t p_ave;          // average encoder pulse value
    double p_kp = 0;        //
    double p_ki = 0;        //
    double p_kd = 0;        //
    double v_ins;           // instant motor velocity in m/s
    double v_pre = 0;       // previous motor velocity in m/s
    double v_set = 0;       // setpoint motor velocity in m/s
    double v_ave;           // average motor velocity in m/s
    uint32_t v_cou = 0;     // velocity sample counter
    double v_sum = 0;       // velocity sum
    double v_kp = 3.0;      //
    double v_ki = 0;        //
    double v_kd = 180.0;    //
    double v_Pro = 0;       // velocity proportional
    double v_Int = 0;       // velocity integral
    double v_Der = 0;       // velocity derivative
    double v_ei = 0;        // instant velocity error
    double v_de = 0;        //
    uint32_t timeout = 500; // velocity timeout, after this velocity set to 0
    uint32_t lastcall = 0;  // last time set velocity
    uint32_t last_t;        // last time call in milisecond
    uint32_t last_pid_t;    // last time call in milisecond
    uint32_t DIR_PIN_ADDR;  // direction pin address
    uint32_t PWM_PIN_ADDR;  // pwm pin address
    uint32_t A_PIN_ADDR;    // encoder channel A pin address
    uint32_t B_PIN_ADDR;    // encoder channel B pin address
    String state;           //
    uint32_t controlMode;   // 1 for position control, 2 for velocity control
    uint32_t SampleTime;
    uint32_t controllerDirection;

    struct pid_var
    {
        double kp, ki, kd;
        double outMax, outMin;
        uint32_t last_t;
    } position, velocity;
    void positionCompute()
    {
        double err = this->p_set - this->p_ins;

        double P_Block = this->p_kp * err;
        double I_Block = this->p_ki * 0;
        double D_Block = this->p_kd * 0;

        uint32_t Sum_Voltage = P_Block + I_Block + D_Block;
        this->drive(Sum_Voltage);
    }
    void velocityCompute()
    {
        this->v_Pro = this->v_set - this->v_ins;
        double pid_scale_factor = 1;
        // if (this->v_Pro < 0.1)
        //   pid_scale_factor = 2;
        // else if (this->v_Pro < 0.2)
        //   pid_scale_factor = 1.5;
        this->B_Voltage = this->voltage;
        // if (abs(this->v_Pro) > 0.005)
        //   this->P_Voltage = pid_scale_factor * this->v_kp * this->v_Pro;
        // else
        //   this->P_Voltage = 0;
        this->P_Voltage = pid_scale_factor * this->v_kp * this->v_Pro;
        this->I_Voltage = this->v_ki * this->v_Int;
        this->D_Voltage = this->v_kd * this->v_Der;

        this->v_Int = 0;
        this->v_Der = 0;

        double Sum_Voltage = B_Voltage + P_Voltage + I_Voltage + D_Voltage;
        this->drive(Sum_Voltage);
    }
    void drive(double set_voltage)
    {
        double output_voltage = set_voltage;
        if (output_voltage > 12)
            output_voltage = 12;
        if (output_voltage < -12)
            output_voltage = 12;
        uint32_t duty = abs(output_voltage) * CONFIG.PWM_MAX_VAL / 12;
        this->voltage = output_voltage;
        if (output_voltage > 0)
        {
            digitalWrite(this->DIR_PIN_ADDR, LOW);
            analogWrite(this->PWM_PIN_ADDR, duty);
        }
        else
        {
            digitalWrite(this->DIR_PIN_ADDR, HIGH);
            analogWrite(this->PWM_PIN_ADDR, CONFIG.PWM_MAX_VAL - duty);
        }
    }

public:
    double voltage; //
    double B_Voltage = 0;
    double P_Voltage = 0;
    double I_Voltage = 0;
    double D_Voltage = 0;

    MotorControl(String id, uint32_t direction_pin, uint32_t pwm_pin, uint32_t a_pin, uint32_t b_pin)
    {
        this->id = id;
        this->a = 0;
        this->b = 0;
        this->p_ins = 0;
        this->p_pre = 0;
        this->last_t = millis();
        this->DIR_PIN_ADDR = direction_pin;
        this->PWM_PIN_ADDR = pwm_pin;
        this->state = "float";
        this->controlMode = IF_POSITION_CONTROL_MODE;
    }
    void setVelocityPID(double v_kp, double v_ki, double v_kd)
    {
        this->v_kp = v_kp;
        this->v_ki = v_ki;
        this->v_kd = v_kd;
    }
    void setPositionPID(double p_kp, double p_ki, double p_kd)
    {
        this->p_kp = p_kp;
        this->p_ki = p_ki;
        this->p_kd = p_kd;
    }
    void info()
    {
        // Bridge.print("Motor ");
        // Bridge.print(this->id);
        // Bridge.print("\tVelocity:");
        // Bridge.print(" kp:");
        // Bridge.print(this->v_kp);
        // Bridge.print(" ki:");
        // Bridge.print(this->v_ki);
        // Bridge.print(" kd:");
        // Bridge.println(this->v_kd);
        // Bridge.print("Position: ");
        // Bridge.print(" kp:");
        // Bridge.print(this->p_kp);
        // Bridge.print(" ki:");
        // Bridge.print(this->p_ki);
        // Bridge.print(" kd:");
        // Bridge.println(this->p_kd);
    }
    void tick(uint32_t t)
    {
        // calculate instant velocity
        const int64_t present_p = this->p_ins;
        const uint32_t present_t = millis();
        uint32_t d_t = present_t - this->last_t; // in miliseconds
        this->d_p = present_p - this->p_pre;
        this->v_ins = (this->d_p * 60000.0) / (d_t * CONFIG.MOTOR_MAX_PPR * CONFIG.MPS_TO_RPM_FACTOR);
        this->p_pre = present_p;
        this->last_t = present_t;

        // prepare for calculating everage velocity
        this->v_sum += v_ins;
        this->v_cou++;

        // calculate Integral
        this->v_Int += (this->v_set - this->v_ins) * d_t;
        // calculate derivative
        this->v_Der = (this->v_pre - this->v_ins) / d_t;

        this->v_pre = this->v_ins;

        // check pid cycle
        if (present_t - this->last_pid_t >= CONFIG.PID_SAMPLE_CYCLE)
        {
            this->last_pid_t = present_p;
            // pid computing
            pidCompute();
            // calculate average volocity
            this->v_ave = this->v_sum / this->v_cou;
            this->v_sum = 0;
            this->v_cou = 0;
        }
    }
    void pidCompute()
    {
        switch (this->controlMode)
        {
        case IF_POSITION_CONTROL_MODE: // Control position mode
            this->positionCompute();
            break;
        case IF_VELOCITY_CONTROL_MODE: // Control velocity mode
            this->velocityCompute();
            break;
        case IF_LOCKED_MODE: // Locked mode
            break;
        default:
            break;
        }
    }
    void step_fw(int32_t c)
    {
        this->a += c;
        this->p_ins += c;
    }
    void step_bw(int32_t c)
    {
        this->b += c;
        this->p_ins += c;
    }
    void reset()
    {
        this->a = 0;
        this->b = 0;
        this->p_ins = 0;
    }
    int64_t getPosition()
    {
        return this->p_ins;
    }
    double getVelocity()
    {
        return this->v_ins;
    }
    double getSetVelocity()
    {
        return this->v_set;
    }
    double getAverageVelocity()
    {
        return this->v_ave;
    }
    double getSpeed()
    {
        return this->v_ins * CONFIG.MPS_TO_RPM_FACTOR;
    }
    int32_t getdp()
    {
        return this->d_p;
    }
    void setPosition(int32_t p)
    {
        this->p_set = p;
        this->controlMode = IF_POSITION_CONTROL_MODE;
    }
    void setVelocity(double v)
    {
        if (v > CONFIG.MOTOR_MAX_SPEED_IN_MPS)
            return;
        this->v_set = v;
        this->voltage = this->v_set * CONFIG.MPS_TO_RPM_FACTOR * 12 / CONFIG.MOTOR_MAX_SPEED_IN_RPM;
        this->controlMode = IF_VELOCITY_CONTROL_MODE;
        this->lastcall = millis();
    }
    void lock()
    {
        setPosition(this->p_ins);
    }
    void stop()
    {
        this->drive(0.0);
        this->controlMode = IF_LOCKED_MODE;
    }
} motor1("M-1", MOTOR_1_DIR, MOTOR_1_PWM, MOTOR_1_A, MOTOR_1_B),
    motor2("M-2", MOTOR_2_DIR, MOTOR_2_PWM, MOTOR_2_A, MOTOR_2_B),
    motor3("M-3", MOTOR_3_DIR, MOTOR_3_PWM, MOTOR_3_A, MOTOR_3_B),
    motor4("M-4", MOTOR_4_DIR, MOTOR_4_PWM, MOTOR_4_A, MOTOR_4_B);
