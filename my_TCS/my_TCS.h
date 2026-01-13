// my_TCS.h - เวอร์ชันสมบูรณ์ล่าสุด
#ifndef my_TCS_h
#define my_TCS_h

#include <Wire.h>

#define EXTERNAL_EEPROM_ADDR  0x50
#define COLOR_CAL_START_ADDR  100

class my_TCS {
public:
    bool begin(uint8_t addr = 0x29);
    bool readFast();

    uint16_t c, r, g, b;
    uint8_t R() { return rgb_r; }
    uint8_t G() { return rgb_g; }
    uint8_t B() { return rgb_b; }
    uint16_t lux() { return calculateLux(r, g, b); }

    void calibrateRed();
    void calibrateGreen();
    void calibrateBlue();
    void calibrateYellow();
    void calibrateWhite();
    void calibrateBlack();
    void calibrateDone();

    void saveColorCalibration();
    void loadColorCalibration();

    bool isCalibrated() { return calibrated; }
    String getColor();

private:
    uint8_t _addr;

    uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);
    void write8(uint8_t reg, uint8_t val);
    uint16_t read16(uint8_t reg);

    void writeEEPROM(uint16_t eeaddress, uint8_t data);
    uint8_t readEEPROM(uint16_t eeaddress);
    void writeEEPROM16(uint16_t eeaddress, uint16_t data);
    uint16_t readEEPROM16(uint16_t eeaddress);

    uint8_t rgb_r, rgb_g, rgb_b;

    bool cal_red = false, cal_green = false, cal_blue = false;
    bool cal_yellow = false, cal_white = false, cal_black = false;
    bool calibrated = false;

    float ref_ratio_r[6] = {0.0};
    float ref_ratio_g[6] = {0.0};
    float ref_ratio_b[6] = {0.0};  // 0=RED, 1=GREEN, 2=BLUE, 3=YELLOW, 4=WHITE, 5=BLACK
};

#endif