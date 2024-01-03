#include "DFRobot_INA219.h"

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);

class PowerMeter
{
private:
    float ina219Reading_mA = 1000;
    float extMeterReading_mA = 1000;

public:
    PowerMeter(/* args */){};
    ~PowerMeter(){};
    int init()
    {
        while (ina219.begin() != true)
        {
            Serial.println("INA219 begin faild");
            delay(2000);
        }
        ina219.linearCalibrate(/*The measured current before calibration*/ ina219Reading_mA, /*The current measured by other current testers*/ extMeterReading_mA);
        Serial.println();
        return 1;
    }
    void info(Stream &stream)
    {
        stream.print("BusVoltage:   ");
        stream.print(ina219.getBusVoltage_V(), 2);
        stream.println("V");
        stream.print("ShuntVoltage: ");
        stream.print(ina219.getShuntVoltage_mV(), 3);
        stream.println("mV");
        stream.print("Current:      ");
        stream.print(ina219.getCurrent_mA(), 1);
        stream.println("mA");
        stream.print("Power:        ");
        stream.print(ina219.getPower_mW(), 1);
        stream.println("mW");
        stream.println("");
    }
} dfMeter;