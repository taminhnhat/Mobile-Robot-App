#include <Arduino.h>
#include <CRC32.h>

CRC32 crc;
uint32_t checksum(String str)
{
    str.trim();
    for (char &c : str)
    {
        crc.update(c);
    }
    uint32_t cres = crc.finalize();
    crc.reset();
    return cres;
}