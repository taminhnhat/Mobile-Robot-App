#include <oled.h>

OLED display(PB7, PB6, NO_RESET_PIN, OLED::W_128, OLED::H_32, OLED::CTRL_SSD1306, 0x3C);

void lcdRender()
{
    display.clear();
    String Amp = String() + 4.5 + "A";
    display.drawString(7, 0, Amp.c_str(), OLED::NORMAL_SIZE, OLED::WHITE);
    String Wat = String() + 19.1 + "W";
    display.drawString(14, 0, Wat.c_str(), OLED::NORMAL_SIZE, OLED::WHITE);
    String Vol = String() + battery.getAverageVoltage() + "V";
    display.drawString(0, 2, Vol.c_str(), OLED::DOUBLE_SIZE, OLED::WHITE);
    uint8_t batteryScale = 42.0 * (battery.getAverageVoltage() - 12.8) / 4;
    display.draw_rectangle(80, 15, 125, 31, OLED::HOLLOW, OLED::WHITE);
    display.draw_rectangle(125, 18, 127, 27, OLED::SOLID, OLED::WHITE);
    display.draw_rectangle(82, 17, 81 + batteryScale, 29, OLED::SOLID, OLED::WHITE);
    display.display();
}