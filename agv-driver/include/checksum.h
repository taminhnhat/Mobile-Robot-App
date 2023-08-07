#include <Arduino.h>
#include <CRC32.h>

CRC32 crc;

uint32_t crc_generate(String str)
{
    uint32_t begin_idx = str.indexOf('{');                              //
    uint32_t end_idx = str.indexOf('}');                                //
    String str_to_generate_crc = str.substring(begin_idx, end_idx + 1); //
    for (char &c : str_to_generate_crc)
    {
        crc.update(c);
    }
    uint32_t cal_cs = crc.finalize();
    crc.reset();
    return cal_cs;
}