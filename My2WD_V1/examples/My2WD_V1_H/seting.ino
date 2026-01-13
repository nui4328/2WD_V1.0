

void _set()
  {
    set_move_before_moveLR(retreat_distance);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน
    Acceptable_values_moveLR(1.0,  1);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน 
    pinMode(buttonPin, INPUT_PULLUP);  
    servo(20, servo20);
    servo(27, servo27);
    servo(28, servo28);
    pinMode(0, INPUT_PULLUP);
    sensor_set();
    color_begin();

    color.loadColorCalibration();  // โหลดค่าที่เคย calibrate ไว้
    bz(100);
    bz(100);
    showSplash();
    runMenuSystem();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)
  // test_color();      // ทดสอบอ่านค่าสีพื้น
  }
 
