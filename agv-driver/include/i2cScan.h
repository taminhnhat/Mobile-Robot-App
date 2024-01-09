#include <Wire.h>

void scan(Stream &stream)
{
    byte error, address;
    int nDevices;
    stream.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);

        error = Wire.endTransmission();

        if (error == 0)
        {
            stream.print("I2C device found at address 0x");
            if (address < 16)
                stream.print("0");
            stream.print(address, HEX);
            stream.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            stream.print("Unknown error at address 0x");
            if (address < 16)
                stream.print("0");
            stream.println(address, HEX);
        }
    }
    if (nDevices == 0)
        stream.println("No I2C devices found\n");
    else
        stream.println("done\n");
}