// ขา PWM สำหรับ servo 1-6
const int servoPins[] = {34, 35, 36, 37, 38, 39};

// ความถี่ PWM สำหรับเซอร์โว
const int SERVO_FREQ = 50; // 50Hz = 20ms period

// ช่วงพัลส์เซอร์โว (0° = ~500us, 180° = ~2500us)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

void servo_set()
  {
    // ตั้งค่าความถี่และความละเอียด PWM
  analogWriteFreq(SERVO_FREQ);       // ตั้งความถี่ PWM เป็น 50Hz
  analogWriteResolution(16);         // ใช้ความละเอียด 16 บิต

  // ขาเซอร์โว 34–39
  for (int pin = 34; pin <= 39; pin++) {
    pinMode(pin, OUTPUT);
  }
  }

// ฟังก์ชันควบคุมเซอร์โวด้วย GPIO โดยตรง
void servo(int pin, int angle) {
  if (pin < 0 || pin > 39) return;
  angle = constrain(angle, 0, 180); // ป้องกันมุมเกิน
  int pulse_us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US); // คำนวณความกว้างพัลส์

  // คำนวณ duty cycle สำหรับ 16-bit analogWrite
  int duty = map(pulse_us, 0, 20000, 0, 65535); // 20ms = 50Hz

  analogWrite(pin, duty);
}