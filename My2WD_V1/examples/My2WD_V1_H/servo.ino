void servo_red()
  {   
    red_box = 1;
    bz(100);
    servo(27, servo27-45);
    delay(800);
    servo(27, servo27);
     //------------->>  ตรวจสอบนับการวางกล่องสี
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo(20, servo20-90);
        bz(300);bz(300);bz(300);
        runMenuSystem();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN) 
      }
    delay(200);
  }

void servo_green()
  {
    green_box = 1;
    bz(100);bz(100);
    servo(27, servo27+45);
    delay(800);
    servo(27, servo27);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo(20, servo20-90);
        bz(300);bz(300);bz(300);
        runMenuSystem();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)  
      }
    delay(200);
  }

void servo_yellow()
  {
    yello_box = 1;
    bz(50);bz(50);bz(50);bz(50);
    servo(28, servo28+45);
    delay(800);
    servo(28, servo28);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo(20, servo20-90);
        bz(300);bz(300);bz(300);
        runMenuSystem();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)     
      }
    delay(200);
  }
void servo_blue()
  {
    blue_box = 1;
    bz(100);bz(100);bz(100);
    servo(28, servo28-45);
    delay(800);
    servo(28, servo28);
    if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
      {
        servo(20, servo20-90);
        bz(300);bz(300);bz(300);
        runMenuSystem();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)       
      }
     delay(200);
  }
