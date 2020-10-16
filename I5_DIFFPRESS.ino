void Get_diff_pressure() {
  float sensor_volt;
  float sensorValue = 0;
  float pressure_value_PSI = 0;
  float pressure_range_PSI = 0.15;
  int pressure_value_PA = 0;
  int pressure_range_PA = 1034;

  for(int x = 0; x <= 100; x++){
    sensorValue = sensorValue + analogRead(A0);
  }
  sensorValue = sensorValue/100.0;
  sensor_volt = sensorValue/1024*5.0;
  pressure_value_PSI = (sensor_volt - 0.5) * (pressure_range_PSI / (5 * 0.8));
  pressure_value_PA = (sensor_volt - 0.5) * (pressure_range_PA / (5 * 0.8));

  delay(1000);
}
