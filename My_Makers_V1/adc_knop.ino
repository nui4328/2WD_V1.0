int knopRead()
  {
    long ADC01 = 0;
    int adc_01;

    // อ่านจาก Wire (I2C0)
    Wire.requestFrom(MCP3421_ADDR, 4);
    if (Wire.available() == 4) 
      {
        byte b1 = Wire.read();
        byte b2 = Wire.read();
        byte b3 = Wire.read();
        byte cfg = Wire.read();

        ADC01 = ((long)b1 << 16) | ((long)b2 << 8) | b3;
        if (b1 & 0x80) ADC01 |= 0xFF000000; // sign-extend
      }
    adc_01 = map(ADC01, 10, 131100, 4000, 0);
    //Serial.print("ADC01: "); Serial.print(adc_01);
    //delay(10);
    return adc_01;
  }