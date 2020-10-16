void Set_status(byte mode) {
  if (PM25_val > PM1) {
    if (PM25_val > PM2) {
      if (PM25_val > PM3) {
        if (PM25_val > PM4) {
          if (PM25_val > PM5) {
            // LED_RED , Auto mode => Motor speed = 6
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_G, LOW);
            digitalWrite(LED_B, HIGH);
            if (mode == 0x02) {
              motor_speed = 0x06;
              cmd90[4] = motor_speed;
            }
          }
          else {
            // LED_MAGENTA , Auto mode => Motor speed = 5
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_G, LOW);
            digitalWrite(LED_B, HIGH);
            if (mode == 0x02) {
              motor_speed = 0x05;
              cmd90[4] = motor_speed;
            }
          }
        }
        else {
          // LED_YELLOW , Auto mode => Motor speed = 4
          digitalWrite(LED_R, LOW);
          digitalWrite(LED_G, LOW);
          digitalWrite(LED_B, HIGH);
          if (mode == 0x02) {
            motor_speed = 0x04;
            cmd90[4] = motor_speed;
          }
        }
      }
      else {
        // LED_GREEN , Auto mode => Motor speed = 3
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_B, HIGH);
        if (mode == 0x02) {
          motor_speed = 0x03;
          cmd90[4] = motor_speed;
        }
      }
    }
    else {
      // LED_CYAN , Auto mode => Motor speed = 2
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, HIGH);
      if (mode == 0x02) {
        motor_speed = 0x02;
        cmd90[4] = motor_speed;
      }
    }
  }
  else {
    // LED_BLUE , Auto mode => Motor speed = 1
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
    if (mode == 0x02) {
      motor_speed = 0x01;
      cmd90[4] = motor_speed;
    }
  }
}
