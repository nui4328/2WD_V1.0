// my_TCS.cpp - เวอร์ชันสมบูรณ์สุด (ปรับปรุงล่าสุด 06 ม.ค. 2026)
// ใช้ค่า EEPROM อัตโนมัติ + เสถียรสูงสุดแม้ calibrate ใหม่ทุกครั้ง

#include "my_TCS.h"
#include <Arduino.h>

bool my_TCS::begin(uint8_t addr) {
    _addr = addr;
    Wire.begin();
    
    uint8_t id = read16(0x12) & 0xFF;
    if (id != 0x44 && id != 0x4D) return false;

    write8(0x00, 0x01);
    delay(3);
    write8(0x00, 0x03);

    write8(0x01, 0xFF);    // Integration time เร็วสุด (2.4ms)
    write8(0x0F, 0x03);    // Gain สูงสุด (60x)

    calibrated = false;
    return true;
}

bool my_TCS::readFast() {
    Wire.beginTransmission(_addr);
    Wire.write(0xA0 | 0x13);
    Wire.endTransmission();
    Wire.requestFrom(_addr, (uint8_t)1);
    if (Wire.available() < 1) return false;
    if (!(Wire.read() & 0x01)) return false;

    c = read16(0x14);
    r = read16(0x16);
    g = read16(0x18);
    b = read16(0x1A);

    if (c == 0) {
        rgb_r = rgb_g = rgb_b = 0;
    } else {
        float factor = 255.0f / c;
        rgb_r = constrain((uint16_t)(r * factor), 0, 255);
        rgb_g = constrain((uint16_t)(g * factor), 0, 255);
        rgb_b = constrain((uint16_t)(b * factor), 0, 255);
    }
    return true;
}

// === ฟังก์ชัน Calibrate (คงเดิมทั้งหมด) ===
void my_TCS::calibrateRed() {
    readFast(); delay(50); readFast();
    ref_ratio_r[0] = (float)r / c;
    ref_ratio_g[0] = (float)g / c;
    ref_ratio_b[0] = (float)b / c;
    cal_red = true;
    Serial.println("=== Calibrated RED ===");
    Serial.print("Raw: C="); Serial.print(c); Serial.print(" R="); Serial.print(r); Serial.print(" G="); Serial.print(g); Serial.print(" B="); Serial.println(b);
    Serial.print("Ratio: R="); Serial.print(ref_ratio_r[0], 4); Serial.print(" G="); Serial.print(ref_ratio_g[0], 4); Serial.print(" B="); Serial.println(ref_ratio_b[0], 4);
}

void my_TCS::calibrateGreen() {
    readFast(); delay(50); readFast();
    ref_ratio_r[1] = (float)r / c;
    ref_ratio_g[1] = (float)g / c;
    ref_ratio_b[1] = (float)b / c;
    cal_green = true;
    Serial.println("=== Calibrated GREEN ===");
    Serial.print("Raw: C="); Serial.print(c); Serial.print(" R="); Serial.print(r); Serial.print(" G="); Serial.print(g); Serial.print(" B="); Serial.println(b);
    Serial.print("Ratio: R="); Serial.print(ref_ratio_r[1], 4); Serial.print(" G="); Serial.print(ref_ratio_g[1], 4); Serial.print(" B="); Serial.println(ref_ratio_b[1], 4);
}

void my_TCS::calibrateBlue() {
    readFast(); delay(50); readFast();
    ref_ratio_r[2] = (float)r / c;
    ref_ratio_g[2] = (float)g / c;
    ref_ratio_b[2] = (float)b / c;
    cal_blue = true;
    Serial.println("=== Calibrated BLUE ===");
    Serial.print("Raw: C="); Serial.print(c); Serial.print(" R="); Serial.print(r); Serial.print(" G="); Serial.print(g); Serial.print(" B="); Serial.println(b);
    Serial.print("Ratio: R="); Serial.print(ref_ratio_r[2], 4); Serial.print(" G="); Serial.print(ref_ratio_g[2], 4); Serial.print(" B="); Serial.println(ref_ratio_b[2], 4);
}

void my_TCS::calibrateYellow() {
    readFast(); delay(50); readFast();
    ref_ratio_r[3] = (float)r / c;
    ref_ratio_g[3] = (float)g / c;
    ref_ratio_b[3] = (float)b / c;
    cal_yellow = true;
    Serial.println("=== Calibrated YELLOW ===");
    Serial.print("Raw: C="); Serial.print(c); Serial.print(" R="); Serial.print(r); Serial.print(" G="); Serial.print(g); Serial.print(" B="); Serial.println(b);
    Serial.print("Ratio: R="); Serial.print(ref_ratio_r[3], 4); Serial.print(" G="); Serial.print(ref_ratio_g[3], 4); Serial.print(" B="); Serial.println(ref_ratio_b[3], 4);
}

void my_TCS::calibrateWhite() {
    readFast(); delay(50); readFast();
    ref_ratio_r[4] = (float)r / c;
    ref_ratio_g[4] = (float)g / c;
    ref_ratio_b[4] = (float)b / c;
    cal_white = true;
    Serial.println("=== Calibrated WHITE ===");
    Serial.print("Raw: C="); Serial.print(c); Serial.print(" R="); Serial.print(r); Serial.print(" G="); Serial.print(g); Serial.print(" B="); Serial.println(b);
    Serial.print("Ratio: R="); Serial.print(ref_ratio_r[4], 4); Serial.print(" G="); Serial.print(ref_ratio_g[4], 4); Serial.print(" B="); Serial.println(ref_ratio_b[4], 4);
}

void my_TCS::calibrateBlack() {
    readFast(); delay(50); readFast();
    ref_ratio_r[5] = (float)r / c;
    ref_ratio_g[5] = (float)g / c;
    ref_ratio_b[5] = (float)b / c;
    cal_black = true;
    Serial.println("=== Calibrated BLACK ===");
    Serial.print("Raw: C="); Serial.print(c); Serial.print(" R="); Serial.print(r); Serial.print(" G="); Serial.print(g); Serial.print(" B="); Serial.println(b);
    Serial.print("Ratio: R="); Serial.print(ref_ratio_r[5], 4); Serial.print(" G="); Serial.print(ref_ratio_g[5], 4); Serial.print(" B="); Serial.println(ref_ratio_b[5], 4);
}

void my_TCS::calibrateDone() {
    calibrated = true;
    Serial.println(">>> All colors calibrated & ready!");
}

// === getColor() เวอร์ชันใหม่สุด - ใช้ EEPROM อัตโนมัติ ===
String my_TCS::getColor() {
    if (!calibrated || c < 20) return "UNKNOWN";

    float curr_r = (float)r / c;
    float curr_g = (float)g / c;
    float curr_b = (float)b / c;

    // Euclidean Distance (ตัวสำรอง)
    long minDist = 999999999L;
    int bestColor = -1;
    const String names[6] = {"RED", "GREEN", "BLUE", "YELLOW", "WHITE", "BLACK"};

    for (int i = 0; i < 6; i++) {
        bool thisCal = (i==0 && cal_red) || (i==1 && cal_green) || 
                       (i==2 && cal_blue) || (i==3 && cal_yellow) || 
                       (i==4 && cal_white) || (i==5 && cal_black);
        if (!thisCal) continue;

        long dr = (long)((ref_ratio_r[i] - curr_r) * 10000);
        long dg = (long)((ref_ratio_g[i] - curr_g) * 10000);
        long db = (long)((ref_ratio_b[i] - curr_b) * 10000);
        long distance = dr*dr + dg*dg + db*db;

        if (distance < minDist) {
            minDist = distance;
            bestColor = i;
        }
    }

    // === Rule พิเศษ - เข้มงวดขึ้นมากเพื่อป้องกัน false positive ===
    
    // BLACK
    if (cal_black && c < 120) {
        return "BLACK";
    }

    // WHITE - เข้มงวดสุด ๆ เพื่อไม่ให้หลุดไปเป็นสีอื่น
    if (cal_white && c > 200 &&  // ต้องแสงพอสมควร
        fabs(curr_r - ref_ratio_r[4]) < 0.07 &&  // ลด margin จาก 0.10 → 0.07
        fabs(curr_g - ref_ratio_g[4]) < 0.07 &&
        fabs(curr_b - ref_ratio_b[4]) < 0.07) {
        return "WHITE";
    }

    // YELLOW - ยังคง priority สูง + บังคับ R >= G
    if (cal_yellow &&
        curr_r > ref_ratio_r[3] - 0.12 && curr_r < ref_ratio_r[3] + 0.12 &&
        curr_g > ref_ratio_g[3] - 0.12 && curr_g < ref_ratio_g[3] + 0.12 &&
        curr_b > ref_ratio_b[3] - 0.08 && curr_b < ref_ratio_b[3] + 0.08 &&
        curr_r >= curr_g - 0.06) {
        return "YELLOW";
    }

    // RED - เข้มงวดขึ้น: R ต้องเด่นชัดกว่าที่ calibrate มาก
    if (cal_red && 
        curr_r > ref_ratio_r[0] - 0.08 &&  // ลด margin จาก 0.12 → 0.08
        curr_r > curr_g + 0.15 &&         // R ต้องนำ G ชัดเจน
        curr_r > curr_b + 0.15) {         // R ต้องนำ B ชัดเจน
        return "RED";
    }

    // BLUE - เข้มงวดขึ้นมากเพื่อไม่ให้พื้นขาวหลุดมา
    if (cal_blue && 
        curr_b > ref_ratio_b[2] - 0.08 &&  // ลด margin
        curr_b > curr_r + 0.12 &&         // B ต้องนำ R ชัดเจน
        curr_b > curr_g + 0.10) {         // B ต้องนำ G
        return "BLUE";
    }

    // GREEN - ยังคงเข้มงวดเหมือนเดิม
    if (cal_green && 
        curr_g > ref_ratio_g[1] - 0.10 &&
        curr_g > curr_r + 0.10 &&
        curr_g > curr_b + 0.08) {
        return "GREEN";
    }

    // === Euclidean สำรอง - ผ่อนปรนแต่ไม่มากเกิน ===
    if (minDist > 600000) return "UNKNOWN";  // ลดจาก 800000 → 600000 เพื่อความแม่นยำสูงขึ้น

    if (bestColor == -1) return "UNKNOWN";
    return names[bestColor];
}

// === ฟังก์ชัน EEPROM (คงเดิมทั้งหมด) ===
void my_TCS::writeEEPROM(uint16_t eeaddress, uint8_t data) {
    Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
    Wire.write((uint8_t)(eeaddress >> 8));
    Wire.write((uint8_t)(eeaddress & 0xFF));
    Wire.write(data);
    Wire.endTransmission();
    delay(5);
}

uint8_t my_TCS::readEEPROM(uint16_t eeaddress) {
    Wire.beginTransmission(EXTERNAL_EEPROM_ADDR);
    Wire.write((uint8_t)(eeaddress >> 8));
    Wire.write((uint8_t)(eeaddress & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(EXTERNAL_EEPROM_ADDR, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0;
}

void my_TCS::writeEEPROM16(uint16_t eeaddress, uint16_t data) {
    writeEEPROM(eeaddress, data >> 8);
    writeEEPROM(eeaddress + 1, data & 0xFF);
}

uint16_t my_TCS::readEEPROM16(uint16_t eeaddress) {
    uint16_t high = readEEPROM(eeaddress);
    uint16_t low = readEEPROM(eeaddress + 1);
    return (high << 8) | low;
}

void my_TCS::saveColorCalibration() {
    uint16_t addr = COLOR_CAL_START_ADDR;

    uint8_t flags = 0;
    if (cal_red)    flags |= (1 << 0);
    if (cal_green)  flags |= (1 << 1);
    if (cal_blue)   flags |= (1 << 2);
    if (cal_yellow) flags |= (1 << 3);
    if (cal_white)  flags |= (1 << 4);
    if (cal_black)  flags |= (1 << 5);
    writeEEPROM(addr++, flags);

    for (int i = 0; i < 6; i++) {
        writeEEPROM16(addr, (uint16_t)(ref_ratio_r[i] * 10000)); addr += 2;
        writeEEPROM16(addr, (uint16_t)(ref_ratio_g[i] * 10000)); addr += 2;
        writeEEPROM16(addr, (uint16_t)(ref_ratio_b[i] * 10000)); addr += 2;
    }

    Serial.println("บันทึกค่า calibrate ลง EEPROM เรียบร้อย!");
}

void my_TCS::loadColorCalibration() {
    uint16_t addr = COLOR_CAL_START_ADDR;

    uint8_t flags = readEEPROM(addr++);
    cal_red    = flags & (1 << 0);
    cal_green  = flags & (1 << 1);
    cal_blue   = flags & (1 << 2);
    cal_yellow = flags & (1 << 3);
    cal_white  = flags & (1 << 4);
    cal_black  = flags & (1 << 5);

    for (int i = 0; i < 6; i++) {
        ref_ratio_r[i] = (float)readEEPROM16(addr) / 10000.0; addr += 2;
        ref_ratio_g[i] = (float)readEEPROM16(addr) / 10000.0; addr += 2;
        ref_ratio_b[i] = (float)readEEPROM16(addr) / 10000.0; addr += 2;
    }

    calibrated = (cal_red || cal_green || cal_blue || cal_yellow || cal_white || cal_black);
    if (calibrated) Serial.println("โหลดค่า calibrate สำเร็จ!");
}

uint16_t my_TCS::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
    return (uint16_t)(-0.32466F * r + 1.57837F * g - 0.73191F * b);
}

void my_TCS::write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(0x80 | reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint16_t my_TCS::read16(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(0xA0 | reg);
    Wire.endTransmission();
    Wire.requestFrom(_addr, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint16_t val = Wire.read();
    val |= (uint16_t)Wire.read() << 8;
    return val;
}