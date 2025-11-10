void Motor(char mt, int pwm) {
  delayMicroseconds(50);
  int pwmValue = map(abs(pwm), 0, 100, 0, 4095); // 0-100% → 0-4095

  int in1, in2;

  if (mt == 'D') { in1 = 2; in2 = 3; }
  else if (mt == 'C') { in1 = 28; in2 = 29; }
  else if (mt == 'B') { in1 = 11; in2 = 10; }
  else if (mt == 'A') { in1 = 6; in2 = 7; }
  else return;

  if (pwm > 0) {
    analogWrite(in1, pwmValue);
    analogWrite(in2, 0);
  } else if (pwm < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, pwmValue);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}
