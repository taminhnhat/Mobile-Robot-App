#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <iostream>
#include "Wire.h"

class IMU
{
private:
    struct vector3
    {
        double x;
        double y;
        double z;
    };
    MPU6050 mpu;
    float euler[3]; // [psi, theta, phi]    Euler angle container
    float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    vector3 gyr;
    vector3 acc;

public:
    IMU()
    {
        //
    }

    int init()
    {
        Wire.begin();
        mpu.initialize();
        if (!mpu.testConnection())
            return -1;
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
        else
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
            return -2;
        }

        return 1;
    }

    int tick()
    {
        if (!dmpReady)
            return -1;
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        }
    }

    void getQuaternion()
    {
        //
    }

    void getLinearAcceleration()
    {
        //
    }

    void getAngularVelocity()
    {
        //
    }
} imu;
