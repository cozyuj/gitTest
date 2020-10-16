/*
   A2
   Compatible with current version of Uni-Q, 2020.07
   Mega board version
*/
#include <String.h>
#include <SoftwareSerial.h>
//#include <SCD30.h>                  // scd30, CO2 sensor
//#include <sps30.h>                  // sps30, particulate material sensor 
//#include <Arduino.h>
//#include <sensirion_arch_config.h>
//#include <sensirion_common.h>
//#include <sensirion_i2c.h>
#include <sps30.h>
//#include <sps_git_version.h>
//#include <I2C.h>
#include <Wire.h>

/* Program Version */
#define sw_version "UNIQ2008AAAJP"

/* Port define */
#define DOOR_PIN  2                 // door switch, interrupt

//#define SP30_COMMS SERIALPORT3      // sps30 comm port, serial3, Particulate material
//#define TX_PIN 14                 // sps30 serial3 tx
//#define RX_PIN 15                 // sps30 serial3 rx

#define zp01_0 4                     // zp01, port 0, tvoc sensor
#define zp01_1 5                     // zp01, port 1

#define a53 A0                      // 53A diff press, analog input
#define rs485_tx 26                 // rs485 tx, software serial
#define rs485_rx 50                 // rs485 rx
#define rs485_mode 28               // rs485 transmit control, half duplex, HIGH for transmit, Low fow receive

#define LED_R  30
#define LED_G  32
#define LED_B  34

#define UV_LED  36
#define ION  38

#define Motor1  9                   // Motor1, pwm
#define Motor2  10                  // Motor2, pwm
#define Motor3  11                  // Motor3, pwm
#define Motor4  12                  // Motor4, pwm
#define Motor_en 39

//#define Motor1_status  40
//#define Motor2_status  42
//#define Motor3_status  44
//#define Motor4_status  46

/* SDS30 sda(20), scl(21)
   RS9A serial1(19, 18)
   reserve for display serial2(17, 16)
*/

int interval = 10000;

byte cmd92[17] = {0xAA, 0x00, 0x92, 0x32, 0x3C, 0x46, 0x50, 0x5A
                  , 0x64, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x38, 0xEE
                 };  // VSP 상태 요구 응답
byte cmd91[17] = {0xAA, 0x00, 0xFF,
                  0x00, 0x00, 0x00, 0x00, bit(7), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0xFF, 0xEE
                 };
byte cmd90[17] = {0xAA, 0x00, 0xFF,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0xFF, 0xEE
                 };

byte air_status = 0x00;

/* global variables */
// for scd 30
float scd30_result[3] = {0};        // scd30_result[0] ==> CO2 concentration, ppm
// scd30_result[1] ==> temparature, ℃
// scd30_result[2] ==> humidity, %

// for rs9a radon sensor
float rs9a_val;                     // 값 (float)
float rs9a_rou;                     // 편자 (float)
int rs9a_rtime;                     // 다음 데이터까지 남은 시간 (int)
int rs9a_unit;                      // 0: pCi/I , 1: Bq/m3 (int)
byte rs9a_reset[7] = {0x52, 0x45, 0x53, 0x45, 0x54, 0x0D, 0x0A};        // rs9a reset command
byte rs9a_unit_c[8] = {0x55, 0x4E, 0x49, 0x54, 0x20, 0x31, 0x0D, 0x0A}; // rs9a unit change command
float rs9a[360] = {0};
float rs9a_avr = 0;
int index = 0;

// for sps30
//SPS30 sps30;                        // assign SPS30 obj to sps30
int PM25_val;                       // PM2.5 value define for display
bool sps30_status = true;

// for zp01
int voc = 0;                        // GAS result return value

// for rs485
SoftwareSerial rs485Serial(rs485_rx, rs485_tx);
byte cmd_buffer[20];                // read/write buffer for serial communication

// for door
boolean interrupt_flag;
volatile boolean door_open; // door open/close status, if door is close, its true
volatile unsigned long cur;
volatile unsigned long pre;

// for motor
volatile byte motor_speed = 0x00;          // Motor speed, interrupt에 의한 속도 조절 필요
volatile byte motor_speed_pre;      // Motor speed befor door open interrupt

// etc
byte filter_status;                 // filter status
byte machine_status;                // machine status, not use but for compatibility

// 센서 기준값
#define PM1  15                  // particulate material PM 2.5
#define PM2  25
#define PM3  35
#define PM4  55
#define PM5  75
#define CO2_1  400                // CO2
#define CO2_2  700
#define CO2_3  800
#define CO2_4  900
#define CO2_5  1000
#define RA_HIGH  148                // Radon Bq.

/*
   Init sensor - initialize input sensors
*/
//void Init_sensors() {
//  bool sps_state;
//  delay(1000);
//  Serial.println("100: setup starts...");
//  /* sps 30 */
//  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN, TX_PIN);
//  //sps_state = sps30.begin(SP30_COMMS);
//  if (sps30.begin(SP30_COMMS) == false) {   // Begin communication channel;
//    Serial.println("200: could not initialize sps30 comm. channel.");
//  }
//  //sps_state = sps30.probe();
//  if (sps30.probe() == false) {
//    Serial.println("300: SPS30, could not probe/connect");
//  } else
//    Serial.println(F("Detected SPS30."));
//  //sps_state = sps30.reset();
//  if (sps30.reset() == false) {
//    Serial.println("310: SPS30, could not reset.");
//  }
//  //sps_state = sps30.start();
//  if (sps30.start() == true)      // start sps30 measurement
//    Serial.println(F("SPS30, Measurement started."));
//  else
//    Serial.println("320: SPS30, Could NOT start measurement.");
//}

/*
   Door, acttion definition for door open/close
*/
void Door() {
  door_open = digitalRead(DOOR_PIN);
  cur = millis();
  if ((cur - pre) > 30) {
    if ((door_open == true) && (interrupt_flag == true)) {
      Serial.println("door_open = true");
      motor_speed_pre = motor_speed;
      motor_speed = 0;
      cmd90[3] = 0x00;
      cmd90[4] = motor_speed;
      cmd90[5] = 0x01;
      cmd90[7] = bit(4);
      digitalWrite(UV_LED, LOW);
      digitalWrite(ION, LOW);
      //delay(100);
      digitalWrite(Motor_en, HIGH);
      interrupt_flag = false;
    }
    else if ((door_open == false) && (interrupt_flag == false)) {
      Serial.println("door_open = false");
      motor_speed = motor_speed_pre;
      cmd90[3] = 0x00;
      cmd90[4] = motor_speed;
      cmd90[5] = 0x01;
      cmd90[7] = bit(7);
      digitalWrite(Motor_en, LOW);
      //delay(100);
      digitalWrite(UV_LED, HIGH);
      digitalWrite(ION, HIGH);
      interrupt_flag = true;
    }
    pre = cur;
  }
}

/*
   Setup
*/
void setup() {
  /* pin setup */
  Wire.begin();
  Serial.begin(9600);               // for debug
  while (!Serial) {
    delay(100);
    Serial.println("fail");
  }

  sensirion_i2c_init();
  
  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
  }
  int16_t ret;
  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }

  rs485Serial.begin(9600);          // for rs485, software serial

  pinMode(rs485_tx, OUTPUT);
  pinMode(rs485_rx, INPUT);
  pinMode(rs485_mode, OUTPUT);

  pinMode(zp01_0, INPUT);           // tvoc input 1
  pinMode(zp01_1, INPUT);           // tvoc input 2

  pinMode(Motor1, OUTPUT);          // motor1
  pinMode(Motor2, OUTPUT);          // motor2
  pinMode(Motor3, OUTPUT);          // motor3
  pinMode(Motor4, OUTPUT);          // motor4
  pinMode(Motor_en, OUTPUT);
  digitalWrite(Motor_en, LOW);
  delay(100);
  analogWrite(Motor1, 255);
  analogWrite(Motor2, 255);
  analogWrite(Motor3, 255);
  analogWrite(Motor4, 255);
  delay(100);
  digitalWrite(Motor_en, HIGH);


  pinMode(LED_R, OUTPUT);           // LED_R
  pinMode(LED_G, OUTPUT);           // LED_G
  pinMode(LED_B, OUTPUT);           // LED_B
  pinMode(UV_LED, OUTPUT);          // UV_LED
  pinMode(ION, OUTPUT);             // ION

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(UV_LED, LOW);
  digitalWrite(ION, LOW);

  pinMode(DOOR_PIN, INPUT);         // sensing door open
  door_open = digitalRead(DOOR_PIN);
  attachInterrupt(digitalPinToInterrupt(DOOR_PIN), Door, CHANGE);
  if (door_open == true) {
    cmd90[3] = 0x00;

    cmd90[4] = 0x00;
    cmd90[5] = 0x01;
    cmd90[7] = bit(4);
    interrupt_flag = false;
    digitalWrite(UV_LED, LOW);
    digitalWrite(ION, LOW);
    digitalWrite(Motor_en, HIGH);
  }
  else {
    cmd90[7] = bit(7);
    interrupt_flag = true;
  }

  /* sensor initialize */
  delay(9000);
  //Init_sensors();
  Set_motor(motor_speed);
  pre = millis();
}

/*
   Set_sensors
*/
void Set_machine(byte *cb) {

}

/*
   Set_VSP
*/
void Set_VSP(byte *cb) {

}

/*
   Set_motor - set motor speed
*/
void Set_motor(byte ms) {
  int a = 0;
  a = ms;
  if (a != 0) {
    analogWrite(Motor1, 225 - ((a - 1) * 14));
    analogWrite(Motor2, 225 - ((a - 1) * 14));
    analogWrite(Motor3, 225 - ((a - 1) * 14));
    analogWrite(Motor4, 225 - ((a - 1) * 14));
  }
  else {
    analogWrite(Motor1, 255);
    analogWrite(Motor2, 255);
    analogWrite(Motor3, 255);
    analogWrite(Motor4, 255);
  }
}

/*
   Read_machine_status - read machine status
*/
void Read_machine_status(byte *rb) {

}

/*
   Cal_cs - calculate checksum, return byte
*/
byte Cal_cs(byte *aa) {
  byte cs = aa[0];
  for (int i = 1; i < 15; i++) {
    cs = cs ^ aa[i];
  }
  return (cs);
}

/*
   Reply - reply for command request
   한컴보드로 reply
*/
void Reply(byte rpy) {
  Serial.println("Call Reply");
  byte rpy_buffer[17] = {0xAA, 0x00, 0xFF,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0xFF, 0xEE
                        };  // reply message

  digitalWrite(rs485_mode, HIGH); // set rs485 for send
  rpy_buffer[2] = rpy;

  switch (rpy) {
    case 0x90:                    // 상태요구 응답
      for (int i = 3; i < 12; i++) {
        rpy_buffer[i] = cmd90[i];
        cmd91[i] = cmd90[i];
      }
      Set_motor(motor_speed);
      break;
    case 0x91:                    // 제어요청 응답
      for (int i = 3; i < 7; i++) {
        rpy_buffer[i] = cmd_buffer[i];
        cmd90[i] = cmd_buffer[i];
      }
      for (int i = 10; i < 12; i++) {
        rpy_buffer[i] = cmd91[i];
      }
      //Read_machine_status(rpy_buffer);
      break;
    case 0x92:                    // vsp 상태요구 응답
      for (int i = 3; i < 15; i++) rpy_buffer[i] = cmd92[i];
      break;
    case 0x93:                    // vsp 제어요청 응답

      for (int i = 3; i < 15; i++) rpy_buffer[i] = cmd92[i];
      break;
    case 0xA0:                    // ID요구 응답
      rpy_buffer[3] = 0x00; rpy_buffer[4] = 0x55; rpy_buffer[5] = 0x4E; rpy_buffer[6] = 0x49;
      rpy_buffer[7] = 0x51; rpy_buffer[8] = 0x10; rpy_buffer[9] = 0x08; rpy_buffer[10] = 0x41;
      rpy_buffer[11] = 0x41; rpy_buffer[12] = 0x041; rpy_buffer[13] = 0x4A; rpy_buffer[14] = 0x50;
      break;
defalut:
      break;
  }
  rpy_buffer[15] = Cal_cs(rpy_buffer);    // check sum
  Serial.print("reply msg = ");
  for (int i = 0; i < 17; i++) {
    Serial.print(rpy_buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  rs485Serial.write(rpy_buffer, 17);    // 응답 전송
  digitalWrite(rs485_mode, LOW);
}

/*
   Read_command
   한컴 보드에서 메가보드로 보내는 명령 해석
*/
void Read_command() {
  door_open = digitalRead(DOOR_PIN);
  int i = 0;
  digitalWrite(rs485_mode, LOW);    // set rs485 for receive
  if (rs485Serial.available()) {
    delay(100);
    Serial.print("receive : ");
    while (rs485Serial.available() && i < 17) {
      cmd_buffer[i] = rs485Serial.read();
      Serial.print(cmd_buffer[i], HEX);
      Serial.print(" ");
      i++;
    }
    Serial.println();

    if ((cmd90[3] == 0x01) && (cmd90[4] != 0x00)) {
      digitalWrite(Motor_en, LOW);
      digitalWrite(ION, HIGH);
      digitalWrite(UV_LED, HIGH);
      Set_status(cmd90[5]);
    } else if (cmd90[3] == 0x00) {
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
      digitalWrite(ION, LOW);
      digitalWrite(UV_LED, LOW);
      motor_speed = 0x00;
      cmd90[4] = motor_speed;
      digitalWrite(Motor_en, HIGH);
    }

    if (cmd_buffer[0] == 0xAA) {
      switch (cmd_buffer[2]) {
        case 0x10:                      // 상태요구
          Set_machine(cmd_buffer);
          Reply(0x90);                  // 상태요구 응답
          break;
        case 0x11:                      // 제어요청
          if (door_open == false) {
            if (cmd_buffer[5] == 0x01)    // 수동 모드
              motor_speed = cmd_buffer[3] * cmd_buffer[4];
            Set_motor(motor_speed);
            Set_machine(cmd_buffer);      // 제어 값 설정 
            
            Reply(0x91);                  // 제어요청 응답
          }
          break;
        case 0x12:                      // VSP 상태요구
          Reply(0x92);                  // VSP 상태요구 응답
          break;
        case 0x13:                      // VSP 제어요구
          Set_VSP(cmd_buffer);          // VSP 제어
          Reply(0x93);                  // VSP 제어요구 응답
          break;
        case 0x20:                      // ID요구
          Reply(0xA0);                  // ID요구 응답
          break;
defalut:
          break;
      } // end switch
    }
  }
}

/*
   Read_sensors - read input sensors value
*/
void Read_sensors() {
  Serial.print("pm = "); Serial.println(PM25_val);
  Get_sps30();                      // paticulate material

  voc = Get_zp01();  // VOC, 0: Clean, 1: Light pollution, 2:Moderate pollution, 3:Severe pollution
  Serial.print("voc = "); Serial.println(voc);
  Serial.println();
}


/*
   Main Loop
*/
void loop() {
  Set_motor(motor_speed);
  Read_sensors();
  //Get_sps30();
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < interval) {
    if (door_open == true) {
      cmd90[3] = 0x00;
      cmd90[4] = 0x00;
      cmd90[5] = 0x01;
    }
    Read_command();
  }
}

// end file

// for debug
void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}
