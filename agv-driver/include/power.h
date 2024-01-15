#include "DFRobot_INA219.h"

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);

class PowerMeter
{
private:
    float ina219Reading_mA = 1000;
    float extMeterReading_mA = 1000;
    double vol, cur, pow = 0;

public:
    PowerMeter(/* args */){};
    ~PowerMeter(){};
    void tick()
    {
        vol = ina219.getBusVoltage_V() * 1000 + ina219.getShuntVoltage_mV();
        cur = ina219.getCurrent_mA();
        pow = ina219.getPower_mW();
    }
    int init(Stream &stream)
    {
        uint8_t c = 1;
        while (ina219.begin() != true)
        {
            if (c >= 3)
            {
                stream.println("INA219 begin fail");
                return 0;
            }
            else
                c++;
            delay(500);
        }
        ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
        stream.println("INA219 begin success");
        return 1;
    }
    void info(Stream &stream)
    {
        stream.print("BusVoltage:   ");
        stream.print(ina219.getBusVoltage_V());
        stream.println("V");
        stream.print("ShuntVoltage: ");
        stream.print(ina219.getShuntVoltage_mV());
        stream.println("mV");
        stream.print("Current:      ");
        stream.print(ina219.getCurrent_mA());
        stream.println("mA");
        stream.print("Power:        ");
        stream.print(ina219.getPower_mW());
        stream.println("mW");
        stream.println("");
    }
    double getVoltage() { return vol / 1000; }
    double getCurrent() { return cur / 1000; }
    double getPower() { return pow / 1000; }
} dfMeter;