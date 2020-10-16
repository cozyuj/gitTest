#include <String.h>

void Get_rs9a() {
  byte request[8] = {0x56, 0x41, 0x4C, 0x55, 0x45, 0x3F, 0x0D, 0x0A}; //value request command
  String rec = "";      // receive string from rs9a
  int valst = 0;
  int valen = 0;
  int roust = 0;
  int rouen = 0;
  int rtimest = 0;
  int rtimeen = 0;
  int unitst = 0;
  int uniten = 0;

  String val_s = "";
  String rou_s = "";
  String rtime_s = "";
  String unit_s = "";

  Serial1.write(request, 8); //rx1, tx1 -> radon_rx, radon_tx
  while (Serial1.available() <= 0) {

  }
  if (Serial1.available() > 0) {
    rec = Serial1.readString();

    valst = rec.indexOf("VALUE");
    valen = rec.indexOf(":ROU");
    roust = rec.indexOf("ROU");
    rouen = rec.indexOf(":rTime");
    rtimest = rec.indexOf("rTime");
    rtimeen = rec.indexOf(":UNIT");
    unitst = rec.indexOf("UNIT");
    uniten = rec.length();

    val_s = rec.substring(valst + 6, valen);      // 값 (string)
    rou_s = rec.substring(roust + 4, rouen);      // 편차 (string)
    rtime_s = rec.substring(rtimest + 6, rtimeen); // 다음 데이터까지 남은 시간 (string)
    unit_s = rec.substring(unitst + 5, uniten);   // 0: pCi/I , 1: Bq/m3 (string)

    rs9a_val = val_s.toFloat();   // 값 (float)
    rs9a_rou = rou_s.toFloat();   // 편자 (float)
    rs9a_rtime = rtime_s.toInt();   // 다음 데이터까지 남은 시간 (int)
    rs9a_unit = unit_s.toInt();     // 0: pCi/I , 1: Bq/m3 (int)

    rs9a[index] = rs9a_val;
    index++;
    if (index == 360) index = 0;
    
    int count = 0;
    for (int i = 0; i < 360; i++){
      if (rs9a[i] != 0){
        count++;
        rs9a_avr = rs9a_avr + rs9a[i];
      }
    }
    rs9a_avr = rs9a_avr / count;
  }
}
