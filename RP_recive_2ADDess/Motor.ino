
int sl, sr; // ตัวแปรความเร็วสำหรับมอเตอร์ซ้ายและขวา

void Motor(int spl, int spr) {
  delayMicroseconds(50); // หน่วงเวลาเพื่อความเสถียร

  // แมปความเร็วจาก -100 ถึง 100 เป็น -4095 ถึง 4095
  sl = map(spl, -100, 100, -4095, 4095);
  sr = map(spr, -100, 100, -4095, 4095);

  // จำกัดค่า
  if (sr > 4095) sr = 4095;
  else if (sr < -4095) sr = -4095;
  if (sl > 4095) sl = 4095; // แก้ไขจาก "sl sl = 4095"
  else if (sl < -4095) sl = -4095;

  // ควบคุมมอเตอร์ซ้าย
  if (sl > 0) {
    digitalWrite(20, HIGH); // AIN1
    digitalWrite(21, LOW);  // AIN2
    analogWrite(3, sl);     // PWMA
  } else if (sl < 0) {
    digitalWrite(20, LOW);
    digitalWrite(21, HIGH);
    analogWrite(3, -sl);
  } else {
    digitalWrite(20, LOW);
    digitalWrite(21, LOW);
    analogWrite(3, 0);
  }

  // ควบคุมมอเตอร์ขวา
  if (sr > 0) {
    digitalWrite(22, HIGH); // BIN1
    digitalWrite(23, LOW);  // BIN2
    analogWrite(6, sr);     // PWMB
  } else if (sr < 0) {
    digitalWrite(22, LOW);
    digitalWrite(23, HIGH);
    analogWrite(6, -sr);
  } else {
    digitalWrite(22, LOW);
    digitalWrite(23, LOW);
    analogWrite(6, 0);
  }


}

