#include <Arduino.h>
#include "configuration.h"
#include <iostream>

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
#define FRONT_LIGHT PC13
#define BATTERY_SENSOR PA1
#define CURRENT_SENSOR_1_2 PB0
#define CURRENT_SENSOR_3_4 PA0
#define MOTOR_EN PB8
#define IMU_INT PB9
#define IMU_SDA PB7
#define IMU_SCL PB6
#define Radio Serial1

HardwareSerial Bridge(PA3, PA2);

void enableMotor()
{
    digitalWrite(MOTOR_EN, HIGH);
}
void disableMotor()
{
    digitalWrite(MOTOR_EN, LOW);
}

double trimDouble(double in, uint8_t num = 2)
{
    if (num == 0)
        return in;
    else
    {
        const uint16_t sc = 10 ^ num;
        return round(in * sc) / sc;
    }
}

class MotorControl
{
private:
#define DIRECT 0
#define REVERSE 1
    // Motor parameters
    String id;
    int64_t a;                 // a channel count
    int64_t b;                 // b channel count
    int64_t p_ins;             // instant encoder pulse value
    int64_t p_pre;             // previous encoder pulse value
    int64_t p_set;             // setpoint encoder pulse value
    int64_t p_ave;             // average encoder pulse value
    int64_t d_p = 0;           //
    double p_kp = 0;           //
    double p_ki = 0;           //
    double p_kd = 0;           //
    double v_ins;              // instant motor velocity in m/s
    double v_pre = 0;          // previous motor velocity in m/s
    double v_set = 0;          // setpoint motor velocity in m/s
    double v_ave;              // average motor velocity in m/s
    double v_sum = 0;          // velocity sum
    uint32_t v_cou = 0;        // velocity sample counter
    double v_kp = 3.0;         //
    double v_ki = 0;           //
    double v_kd = 180.0;       //
    double vol_Pro = 0;        // velocity proportional
    double vol_Int = 0;        // velocity integral
    double vol_Der = 0;        // velocity derivative
    uint32_t timeout = 500;    // velocity timeout, after this velocity set to 0
    uint32_t lastcall = 0;     // last time set velocity
    uint32_t last_t;           // last time call in milisecond
    uint32_t last_pid_t;       // last time call in milisecond
    uint32_t DIR_PIN_ADDR;     // direction pin address
    uint32_t PWM_PIN_ADDR;     // pwm pin address
    uint32_t CUR_SEN_PIN_ADDR; // current sensor pin address
    uint32_t A_PIN_ADDR;       // encoder channel A pin address
    uint32_t B_PIN_ADDR;       // encoder channel B pin address
    String state;              //
    uint32_t controlMode;      // 1 for position control, 2 for velocity control
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
        this->vol_Pro = this->v_set - this->v_ins;
        double pid_scale_factor = 1;
        // if (this->vol_Pro < 0.1)
        //   pid_scale_factor = 2;
        // else if (this->vol_Pro < 0.2)
        //   pid_scale_factor = 1.5;
        this->B_Voltage = this->voltage;
        // if (abs(this->vol_Pro) > 0.005)
        //   this->P_Voltage = pid_scale_factor * this->v_kp * this->vol_Pro;
        // else
        //   this->P_Voltage = 0;
        this->P_Voltage = pid_scale_factor * this->v_kp * this->vol_Pro;
        this->I_Voltage = this->v_ki * this->vol_Int;
        this->D_Voltage = this->v_kd * this->vol_Der;

        this->vol_Int = 0;
        this->vol_Der = 0;

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

    MotorControl(String id, uint32_t direction_pin, uint32_t pwm_pin, uint32_t cur_sen_pin)
    {
        this->id = id;
        this->a = 0;
        this->b = 0;
        this->p_ins = 0;
        this->p_pre = 0;
        this->last_t = millis();
        this->DIR_PIN_ADDR = direction_pin;
        this->PWM_PIN_ADDR = pwm_pin;
        this->CUR_SEN_PIN_ADDR = cur_sen_pin;
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
    void info(Stream &stream)
    {
        stream.print("Motor ");
        stream.print(this->id);
        stream.print("\tVelocity:");
        stream.print(" kp:");
        stream.print(this->v_kp);
        stream.print(" ki:");
        stream.print(this->v_ki);
        stream.print(" kd:");
        stream.print(this->v_kd);
        stream.print("\tPosition: ");
        stream.print(" kp:");
        stream.print(this->p_kp);
        stream.print(" ki:");
        stream.print(this->p_ki);
        stream.print(" kd:");
        stream.println(this->p_kd);
    }
    void tick(uint32_t t)
    {
        // calculate instant velocity
        const int64_t present_p = this->p_ins;
        const uint32_t present_t = millis();
        uint32_t d_t = present_t - this->last_t; // in miliseconds
        this->d_p = present_p - this->p_pre;
        this->v_ins = (this->d_p * 60000.0 * CONFIG.RPM_TO_MPS_FACTOR) / (d_t * CONFIG.MOTOR_PPR);
        this->p_pre = present_p;
        this->last_t = present_t;

        // prepare for calculating everage velocity
        this->v_sum += v_ins;
        this->v_cou++;

        // calculate Integral
        this->vol_Int += (this->v_set - this->v_ins) * d_t;
        // calculate derivative
        this->vol_Der = (this->v_pre - this->v_ins) / d_t;

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
    double getPosition()
    {
        return trimDouble(this->p_ins * CONFIG.PULSE_TO_RAD_FACTOR, 2);
    }
    double getVelocity()
    {
        return trimDouble(this->v_ins, 2);
    }
    double getSetVelocity()
    {
        return this->v_set;
    }
    double getSetSpeed()
    {
        return this->v_set * 2 / CONFIG.WHEEL_DIAMETER;
    }
    double getAverageVelocity()
    {
        return trimDouble(this->v_ave, 2);
    }
    double getSpeed()
    {
        return trimDouble(this->v_ins * 2 / CONFIG.WHEEL_DIAMETER, 2);
    }
    double getAverageSpeed()
    {
        return trimDouble(this->v_ave * 2 / CONFIG.WHEEL_DIAMETER, 2);
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
        if (v > CONFIG.MOTOR_ALLOW_MAX_SPEED_IN_MPS)
            v = CONFIG.MOTOR_ALLOW_MAX_SPEED_IN_MPS;
        if (v < -CONFIG.MOTOR_ALLOW_MAX_SPEED_IN_MPS)
            v = -CONFIG.MOTOR_ALLOW_MAX_SPEED_IN_MPS;
        this->v_set = v;
        this->voltage = this->v_set * 12 / CONFIG.MOTOR_MAX_SPEED_IN_MPS;
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
} motor1("M-1", MOTOR_1_DIR, MOTOR_1_PWM, CURRENT_SENSOR_1_2),
    motor2("M-2", MOTOR_2_DIR, MOTOR_2_PWM, CURRENT_SENSOR_1_2),
    motor3("M-3", MOTOR_3_DIR, MOTOR_3_PWM, CURRENT_SENSOR_3_4),
    motor4("M-4", MOTOR_4_DIR, MOTOR_4_PWM, CURRENT_SENSOR_3_4);

class Battery

{
private:
    double vol_ins;          // instant voltage
    double vol_sum = 0;      // sum voltage
    double vol_ave;          // average voltage
    uint32_t vol_cou;        // voltage sampling count
    uint32_t cal_cyc = 1000; // calculating average voltage cycle
    uint64_t last_t = 0;

public:
    Battery(/* args */) {}
    ~Battery() {}
    void tick()
    {
        this->vol_ins = 3.3 * 5 * analogRead(BATTERY_SENSOR) / 1023;
        this->vol_sum += this->vol_ins;
        this->vol_cou++;
        const uint64_t present_t = millis();
        if (present_t - this->last_t >= this->cal_cyc)
        {
            this->vol_ave = this->vol_sum / this->vol_cou;
            this->vol_sum = 0;
            this->vol_cou = 0;
            this->last_t = present_t;
        }
    }
    double getVoltage()
    {
        return trimDouble(this->vol_ins, 2);
    }
    double getAverageVoltage()
    {
        return trimDouble(this->vol_ave, 2);
    }
} battery;

// class DistanceSensor

// {
// private:
//     double vol_ins;         // instant voltage
//     double dis_ins;         // instant distance
//     double dis_sum = 0;     // sum voltage
//     double dis_ave;         // average voltage
//     uint32_t dis_cou;       // voltage sampling count
//     uint32_t cal_cyc = 100; // calculating average voltage cycle
//     uint64_t last_t = 0;
//     const double vol_factor = 3.3 / 1024;

// public:
//     DistanceSensor(/* args */) {}
//     ~DistanceSensor() {}
//     void tick()
//     {
//         this->vol_ins = analogRead(DISTANCE_SENSOR) * this->vol_factor;

//         if (this->vol_ins > 2.0)
//         {
//             // motor1.stop();
//             // motor2.stop();
//             // motor3.stop();
//             // motor4.stop();
//         }

//         if (this->vol_ins < 0.3)
//         {
//             this->dis_ins = 100;
//         }
//         else if (this->vol_ins < 2.33)
//         {
//             this->dis_ins = 12.5926 / (this->vol_ins + 0.0148) - 0.42;
//         }
//         else if (this->vol_ins < 3.3)
//         {
//             this->dis_ins = 8.5714 / (this->vol_ins - 0.7714) - 0.42;
//         }
//         else
//         {
//             this->dis_ins = 0;
//         }
//         this->dis_sum += this->dis_ins;
//         this->dis_cou++;
//         const uint64_t present_t = millis();
//         if (present_t - this->last_t >= this->cal_cyc)
//         {
//             this->dis_ave = this->dis_sum / this->dis_cou;
//             this->dis_sum = 0;
//             this->dis_cou = 0;
//             this->last_t = present_t;
//         }
//     }
//     double getVoltage()
//     {
//         return this->vol_ins;
//     }
//     double getDistance()
//     {
//         return round(this->dis_ins * 100) / 100;
//     }
//     double getAverageDistance()
//     {
//         return round(this->dis_ave * 100) / 100;
//     }
// } distance;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

struct vector4Double
{
    double x;
    double y;
    double z;
    double w;
};
struct vector3Double
{
    double x;
    double y;
    double z;
};
struct sensors
{
    vector4Double orientation;
    vector3Double angular_velocity;
    vector3Double linear_acceleration;
    double temperature;
};

class IMU
{
private:
    vector4Double orientation;
    vector3Double angular_velocity;
    vector3Double linear_acceleration;
    double temperature;
    MPU6050 mpu;
    float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    VectorInt16 gg;      // [x, y, z]            gyro sensor measurements
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    bool rawMode = true;
    int16_t temp;
    int16_t accAdcScale = 16384.0;
    int16_t gyrAdcScale = 131.072;
    int16_t temAdcScale = 2184.0;
    int16_t resScale = 1000;

public:
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    IMU() {}

    int init(bool raw = true)
    {
        this->rawMode = raw;
        Wire.begin();
        mpu.initialize();
        // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        if (rawMode)
        {
            return 0;
        }
        else
        {
            // enable Digital Motion Processor
            devStatus = mpu.dmpInitialize();

            // supply your own gyro offsets here, scaled for min sensitivity
            mpu.setXGyroOffset(220);
            mpu.setYGyroOffset(76);
            mpu.setZGyroOffset(-85);
            mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

            // make sure it worked (returns 0 if so)
            if (devStatus == 0)
            {
                mpu.CalibrateAccel(6);
                mpu.CalibrateGyro(6);
                mpu.PrintActiveOffsets();
                mpu.setDMPEnabled(true);
                mpuIntStatus = mpu.getIntStatus();
                dmpReady = true;
                packetSize = mpu.dmpGetFIFOPacketSize();
            }
            return devStatus;
        }
    }

    int tick()
    {
        if (rawMode)
        {
            uint8_t buffer[14];
            I2Cdev::readBytes(MPU6050_IMU::MPU6050_DEFAULT_ADDRESS, (MPU6050_IMU::MPU6050_RA_ACCEL_XOUT_H), 14, buffer);
            ax = (((int16_t)buffer[0]) << 8) | buffer[1];
            ay = (((int16_t)buffer[2]) << 8) | buffer[3];
            az = (((int16_t)buffer[4]) << 8) | buffer[5];
            temp = buffer[6] << 8 | buffer[7];
            gx = (((int16_t)buffer[8]) << 8) | buffer[9];
            gy = (((int16_t)buffer[10]) << 8) | buffer[11];
            gz = (((int16_t)buffer[12]) << 8) | buffer[13];
            linear_acceleration.x = round(resScale * ax / accAdcScale) / resScale;
            linear_acceleration.y = round(resScale * ay / accAdcScale) / resScale;
            linear_acceleration.z = round(resScale * az / accAdcScale) / resScale;
            angular_velocity.x = round(resScale * gx / gyrAdcScale) / resScale;
            angular_velocity.y = round(resScale * gy / gyrAdcScale) / resScale;
            angular_velocity.z = round(resScale * gz / gyrAdcScale) / resScale;
            temperature = temp / 340.0 + 36.53;
            // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            // temp = mpu.getTemperature();
            // Serial.print("gyr\t");
            // Serial.print(gx);
            // Serial.print("\t");
            // Serial.print(gy);
            // Serial.print("\t");
            // Serial.print(gz);
            // Serial.print("\tacc\t");
            // Serial.print(ax);
            // Serial.print("\t");
            // Serial.print(ay);
            // Serial.print("\t");
            // Serial.println(az);
        }
        else
        {
            if (!dmpReady)
                return -1;
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
            {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                // mpu.dmpGetGravity(&gravity, &q);
                // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                mpu.dmpGetAccel(&aa, fifoBuffer);
                // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

                mpu.dmpGetGyro(&gg, fifoBuffer);

                orientation.x = q.x;
                orientation.y = q.y;
                orientation.z = q.z;
                orientation.w = q.w;

                angular_velocity.x = gg.x * M_PI / (180 * gyrAdcScale);
                angular_velocity.y = gg.y * M_PI / (180 * gyrAdcScale);
                angular_velocity.z = gg.z * M_PI / (180 * gyrAdcScale);

                linear_acceleration.x = aa.x * 9.81 / accAdcScale;
                linear_acceleration.y = aa.y * 9.81 / accAdcScale;
                linear_acceleration.z = aa.z * 9.81 / accAdcScale;

                temperature = mpu.getTemperature() / 340.0 + 36.53;

                // Serial.print("ORI\t");
                // Serial.print(ori.x);
                // Serial.print("\t");
                // Serial.print(ori.y);
                // Serial.print("\t");
                // Serial.print(ori.z);

                // Serial.print("\tQUA\t");
                // Serial.print(q.x);
                // Serial.print("\t");
                // Serial.print(q.y);
                // Serial.print("\t");
                // Serial.print(q.z);
                // Serial.print("\t");
                // Serial.print(q.w);

                // Serial.print("\tGYR\t");
                // Serial.print(gyr.x);
                // Serial.print("\t");
                // Serial.print(gyr.y);
                // Serial.print("\t");
                // Serial.print(gyr.z);

                // Serial.print("\tACC\t");
                // Serial.print(acc.x);
                // Serial.print("\t");
                // Serial.print(acc.y);
                // Serial.print("\t");
                // Serial.println(acc.z);
            }
        }
        return 0;
    }

    uint16_t getRawGyrX() { return gx; }
    uint16_t getRawGyrY() { return gy; }
    uint16_t getRawGyrZ() { return gz; }

    uint16_t getRawAccX() { return ax; }
    uint16_t getRawAccY() { return ay; }
    uint16_t getRawAccZ() { return az; }

    uint16_t getRawTemperature() { return temp; }

    sensors getFullSensors()
    {
        sensors s;
        s.orientation.x = orientation.x;
        s.orientation.y = orientation.y;
        s.orientation.z = orientation.z;
        s.orientation.w = orientation.w;
        s.angular_velocity.x = angular_velocity.x;
        s.angular_velocity.y = angular_velocity.y;
        s.angular_velocity.z = angular_velocity.z;
        s.linear_acceleration.x = linear_acceleration.x;
        s.linear_acceleration.y = linear_acceleration.y;
        s.linear_acceleration.z = linear_acceleration.z;
        s.temperature = temperature;
        return s;
    }

    double getYaw() { return ypr[0]; }
    double getPitch() { return ypr[1]; }
    double getRoll() { return ypr[2]; }

    double getQuaternionX() { return q.x; }
    double getQuaternionY() { return q.x; }
    double getQuaternionZ() { return q.x; }

    double getLinearAccelerationX() { return aaReal.x; }
    double getLinearAccelerationY() { return aaReal.y; }
    double getLinearAccelerationZ() { return aaReal.z; }

    double getAngularVelocityX() { return 0; }
    double getAngularVelocityY() { return 0; }
    double getAngularVelocityZ() { return 0; }
} imu;
