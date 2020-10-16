int Get_zp01() {
  String gas_Clean = "Clean";
  String gas_Light = "Light pollution";
  String gas_Moderate = "Moderate pollution";
  String gas_Severe = "Severe pollution";
  int a = digitalRead(zp01_0);
  int b = digitalRead(zp01_1);

  if (a == 0 && b == 0) {
    cmd90[11] = 0x01;
    return 0;  // Clean
  }
  else if (a == 0 && b == 1) {
    cmd90[11] = 0x02;
    return 1;  // Light pollution
  }
  else if (a == 1 && b == 0) {
    cmd90[11] = 0x03;
    return 2;  // Moderate pollution
  }
  else {
    cmd90[11] = 0x04;
    return 3;  // Severe pollution
  }
}
